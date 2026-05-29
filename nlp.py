#!/usr/bin/env python3

import os
import re
import socket
import tempfile
import threading
import wave

import pyaudio
import whisper
from pynput import keyboard


# ==================== Laptop UDP configuration ====================
# Change this to the Raspberry Pi's IP address before the live demo.
# Example:
# RPI_IP = "192.168.1.123"
RPI_IP = "10.42.0.1"
RPI_PORT = 9999

WHISPER_MODEL_NAME = "tiny"

# ==================== Audio recording configuration ====================
# Whisper expects 16 kHz mono PCM, which is what we record here directly.
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2
FORMAT = pyaudio.paInt16
CHUNK = 1024

# ==================== Push-to-talk configuration ====================
# Hold this key to record; release to recognize and send the UDP command.
PTT_KEY = keyboard.Key.space
QUIT_KEY = keyboard.Key.esc

# ==================== Supported colors ====================
# Keep this list in sync with FALLBACK_HSV_RANGES in robot_color_follow.py and
# VALID_COLORS in robot_udp_receiver.py.
SUPPORTED_COLORS = ("red", "blue", "green", "yellow", "pink")


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
model = None

ptt_pressed = threading.Event()
quit_event = threading.Event()


def load_whisper_model():
    """Load Whisper on the laptop. The Raspberry Pi never runs Whisper."""
    global model

    if model is None:
        print(f"Loading Whisper model ({WHISPER_MODEL_NAME})...")
        model = whisper.load_model(WHISPER_MODEL_NAME)
        print("Whisper model loaded.")

    return model


def normalize_text(text):
    return re.sub(r"[^a-z0-9\s]", " ", text.lower()).strip()


NUMBER_WORDS = {
    "zero": 0,
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
    "six": 6,
    "seven": 7,
    "eight": 8,
    "nine": 9,
    "ten": 10,
    "eleven": 11,
    "twelve": 12,
    "thirteen": 13,
    "fourteen": 14,
    "fifteen": 15,
    "sixteen": 16,                                   
    "seventeen": 17,
    "eighteen": 18,
    "nineteen": 19,
    "twenty": 20,
    "thirty": 30,
    "forty": 40,
    "fifty": 50,
    "sixty": 60,
    "seventy": 70,
    "eighty": 80,
    "ninety": 90,
}


def number_token_pattern():
    words = "|".join(sorted(NUMBER_WORDS.keys(), key=len, reverse=True))
    return rf"(\d+|{words})"


def parse_number_token(token):
    if token is None:
        return None
    token = token.strip().lower()
    if token.isdigit():
        return int(token)
    return NUMBER_WORDS.get(token)


def parse_motion_commands(normalized):
    """
    Parse simple English manual movement commands.

    Supported examples:
    - "go forward 30 cm"
    - "go straight 30cm"
    - "turn left 90 degree and go straight 30 cm"
    - "turn right 30 degree and go straight 30 cm"
    - "turn clockwise 90 degrees"
    """
    number = number_token_pattern()

    patterns = [
        (
            re.compile(
                rf"\b(?:go|move|drive)\s+(?:forward|straight)\s*{number}\s*(?:cm|centimeter|centimeters)\b"
            ),
            lambda n: f"MOVE_FORWARD_CM_{n}",
        ),
        (
            re.compile(
                rf"\b(?:forward|straight)\s*{number}\s*(?:cm|centimeter|centimeters)\b"
            ),
            lambda n: f"MOVE_FORWARD_CM_{n}",
        ),
        (
            re.compile(
                rf"\bturn\s+left\s*{number}\s*(?:degree|degrees|deg)?\b"
            ),
            lambda n: f"TURN_LEFT_DEG_{n}",
        ),
        (
            re.compile(
                rf"\bturn\s+right\s*{number}\s*(?:degree|degrees|deg)?\b"
            ),
            lambda n: f"TURN_RIGHT_DEG_{n}",
        ),
        (
            re.compile(
                rf"\bturn\s+clockwise\s*{number}\s*(?:degree|degrees|deg)?\b"
            ),
            lambda n: f"TURN_RIGHT_DEG_{n}",
        ),
        (
            re.compile(
                rf"\bclockwise\s+(?:turn|rotate)\s*{number}\s*(?:degree|degrees|deg)?\b"
            ),
            lambda n: f"TURN_RIGHT_DEG_{n}",
        ),
        (
            re.compile(
                rf"\brotate\s+clockwise\s*{number}\s*(?:degree|degrees|deg)?\b"
            ),
            lambda n: f"TURN_RIGHT_DEG_{n}",
        ),
    ]
    matches = []
    for pattern, builder in patterns:
        for match in pattern.finditer(normalized):
            value = parse_number_token(match.group(1))
            if value is None:
                continue

            # Store the span so overlapping matches can be removed later.
            matches.append((match.start(), match.end(), builder(value)))

    if not matches:
        return None

    matches.sort(key=lambda item: (item[0], -(item[1] - item[0])))

    filtered = []
    used_spans = []

    for start, end, command in matches:
        overlaps_existing = any(
            start < used_end and end > used_start
            for used_start, used_end in used_spans
        )

        if overlaps_existing:
            continue

        filtered.append((start, end, command))
        used_spans.append((start, end))

    # Return commands in spoken order.
    filtered.sort(key=lambda item: item[0])
    commands = [command for _, _, command in filtered]

    if len(commands) == 1:
        return commands[0]
    return "SEQ:" + ";".join(commands)


def parse_voice_command(text):
    """
    Parse full English voice commands into robot UDP commands.

    Examples:
    - "follow red", "track the red object", "please follow red" -> FOLLOW_RED
    - "follow blue", "track the blue object", "switch to blue" -> FOLLOW_BLUE
    - "follow green", "track the green object", "switch to green" -> FOLLOW_GREEN
    - "follow yellow", "track the yellow bottle", "switch to yellow" -> FOLLOW_YELLOW
    - "follow pink", "track the pink bottle", "switch to pink" -> FOLLOW_PINK
    - "go forward 30 cm" -> MOVE_FORWARD_CM_30
    - "turn left 90 degree and go straight 30 cm" -> SEQ:TURN_LEFT_DEG_90;MOVE_FORWARD_CM_30
    - "turn right 30 degree and go straight 30 cm" -> SEQ:TURN_RIGHT_DEG_30;MOVE_FORWARD_CM_30
    - "turn clockwise 90 degrees" -> TURN_RIGHT_DEG_90
    - "stop", "stop the robot", "pause", "halt" -> STOP
    """
    normalized = normalize_text(text)
    words = set(normalized.split())

    if words.intersection({"stop", "pause", "halt"}):
        return "STOP"

    motion_command = parse_motion_commands(normalized)
    if motion_command is not None:
        return motion_command

    action_words = {"follow", "following", "track", "tracking", "switch", "switching"}
    if not words.intersection(action_words):
        return None

    for color in SUPPORTED_COLORS:
        if color in words:
            return f"FOLLOW_{color.upper()}"

    return None


def send_udp_command(command):
    """The laptop sends simple UDP command strings to the Raspberry Pi."""
    sock.sendto(command.encode("utf-8"), (RPI_IP, RPI_PORT))
    print(f"Sending command: {command} -> {RPI_IP}:{RPI_PORT}")


def transcribe_and_send(frames):
    """Write the captured PCM frames to a temp wav file and run Whisper."""
    if not frames:
        print("No audio captured; key released too quickly.")
        return

    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_audio:
            temp_path = temp_audio.name

        with wave.open(temp_path, "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(SAMPLE_WIDTH)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes(b"".join(frames))

        print("Recognizing with Whisper...")
        result = load_whisper_model().transcribe(
            temp_path,
            language="en",
            fp16=False,
        )
        text = result["text"].strip()
        print(f"Recognized text: {text}")

        command = parse_voice_command(text)
        if command is None:
            print("No supported robot command found.")
            return

        send_udp_command(command)

    except Exception as exc:
        print(f"Speech recognition error: {exc}")
    finally:
        if temp_path and os.path.exists(temp_path):
            os.remove(temp_path)


def on_press(key):
    if key == PTT_KEY:
        ptt_pressed.set()
    elif key == QUIT_KEY:
        quit_event.set()
        ptt_pressed.set()
        return False


def on_release(key):
    if key == PTT_KEY:
        ptt_pressed.clear()


def push_to_talk_loop():
    audio = pyaudio.PyAudio()

    examples = " | ".join(f"follow {c}" for c in SUPPORTED_COLORS) + " | go forward 30 cm | turn left 90 degree and go straight 30 cm | stop"

    print("\n" + "=" * 56)
    print("Push-to-talk voice control is running on the laptop.")
    print(f"UDP target: {RPI_IP}:{RPI_PORT}")
    print("Hold SPACE to speak. Release to recognize and send.")
    print("Press ESC to quit.")
    print(f"Try: {examples}")
    print("=" * 56)

    try:
        while not quit_event.is_set():
            if not ptt_pressed.wait(timeout=0.2):
                continue
            if quit_event.is_set():
                break

            print("\n[Recording... release SPACE to stop]")

            stream = audio.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=CHUNK,
            )

            frames = []
            try:
                while ptt_pressed.is_set() and not quit_event.is_set():
                    data = stream.read(CHUNK, exception_on_overflow=False)
                    frames.append(data)
            finally:
                stream.stop_stream()
                stream.close()

            duration = len(frames) * CHUNK / SAMPLE_RATE
            print(f"Recorded {duration:.2f} seconds of audio.")

            if duration < 0.3:
                print("Recording too short, ignoring.")
                continue

            transcribe_and_send(frames)

    finally:
        audio.terminate()


def main():
    load_whisper_model()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        push_to_talk_loop()
    except KeyboardInterrupt:
        print("\nStopping voice control...")
    finally:
        quit_event.set()
        listener.stop()
        sock.close()
        print("Quit.")


if __name__ == "__main__":
    main()
