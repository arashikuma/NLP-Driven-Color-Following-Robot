#!/usr/bin/env python3
"""
Voice-controlled color-following robot with optional pre-demo object calibration.

IMPORTANT COLOR CONVENTION
--------------------------
On our current PiCamera2 + OpenCV setup, the captured frame behaves as BGR
for OpenCV. Therefore:
- HSV conversion uses cv2.COLOR_BGR2HSV.
- cv2.imshow uses the captured frame directly.
- Do NOT add RGB->BGR conversion unless your live preview clearly needs it.

This fixes the red/blue channel swap problem.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import picar_4wd as fc
from picamera2 import Picamera2

from robot_udp_receiver import RobotCommandReceiver


# ============================================================
# Frame and loop
# ============================================================
FRAME_W = 640
FRAME_H = 480
LOOP_SLEEP = 0.03

DETECT_W = 160
DETECT_H = 120
KERNEL_5 = np.ones((5, 5), np.uint8)

PROFILE_PATH = Path(__file__).resolve().parent / "object_profiles.json"


# ============================================================
# Motor tuning
# ============================================================
TURN_THRESHOLD = 60
TURN_POWER = 15
FORWARD_POWER = 60
SLOW_FORWARD_POWER = 35

# ============================================================
# Manual voice movement tuning
# ------------------------------------------------------------
# These are time-based approximations. Tune them on your floor/battery.
# Example starting points:
#   30 cm forward  -> 30 * 0.040 = 1.20 seconds
#   90 degree turn -> 90 * 0.012 = 1.08 seconds
# ============================================================
MANUAL_FORWARD_POWER = 45
MANUAL_TURN_POWER = 15
SECONDS_PER_CM = 0.040
SECONDS_PER_DEGREE = 0.012

_manual_queue = []
_manual_current = None
_manual_end_time = 0.0


# ============================================================
# Distance control
# ============================================================
DESIRED_AREA = {
    "red": 50000,
    "blue": 50000,
    "green": 50000,
    "yellow": 30000,
    "pink": 50000,
}

FAR_AREA_RATIO = 0.50
SLOW_AREA_RATIO = 0.85
NEAR_AREA_RATIO = 1.60


# ============================================================
# Switch-to-new-object behavior
# ------------------------------------------------------------
# When a new FOLLOW_COLOR command arrives, the robot:
#   1. stops immediately,
#   2. waits SWITCH_WAIT_SECONDS while checking whether the new object is visible,
#   3. if the new object is visible after the wait, follows it,
#   4. otherwise slowly rotates in place until it finds the new object.
# ============================================================
SWITCH_WAIT_SECONDS = 3.0
SCAN_TURN_POWER = 10

_switch_wait_until = 0.0
_switch_wait_color = None
_switch_scan_active = False


def start_switch_wait(color_name):
    """Start the stop-and-wait phase after receiving a new follow command."""
    global _switch_wait_until, _switch_wait_color, _switch_scan_active
    _switch_wait_until = time.time() + SWITCH_WAIT_SECONDS
    _switch_wait_color = color_name
    _switch_scan_active = False
    stop()


def reset_search_state():
    """Reset switch/search state when robot enters STOP mode."""
    global _switch_wait_until, _switch_wait_color, _switch_scan_active
    _switch_wait_until = 0.0
    _switch_wait_color = None
    _switch_scan_active = False


def handle_switch_wait_or_scan(target, color_name, frame_width=FRAME_W):
    """
    Handle the special behavior immediately after a new object command.

    Return:
        action string if this function handled the current frame;
        None if normal follow_target() should run.
    """
    global _switch_wait_until, _switch_wait_color, _switch_scan_active

    if _switch_wait_color != color_name:
        return None

    now = time.time()

    # Phase 1: stop for 3 seconds and let the camera check for the new object.
    if now < _switch_wait_until:
        stop()
        remaining = max(0.0, _switch_wait_until - now)
        if target is None:
            return f"waiting for {color_name} ({remaining:.1f}s)"
        return f"{color_name} visible, waiting ({remaining:.1f}s)"

    # Phase 2: after waiting, follow if the object is visible.
    if target is not None:
        _switch_wait_until = 0.0
        _switch_wait_color = None
        _switch_scan_active = False
        return None

    # Phase 3: if still not visible, slowly rotate in place to search.
    _switch_scan_active = True
    turn_right(SCAN_TURN_POWER)
    return f"scanning for {color_name}"


# ============================================================
# Fallback HSV detectors
# ------------------------------------------------------------
# Used when no learned object profile exists.
# Red wraps around hue=0, so it uses two ranges.
# ============================================================
FALLBACK_HSV_RANGES = {
    "red": [
        ((0, 60, 60), (4, 255, 255)),
        ((165, 60, 60), (180, 255, 255)),
    ],
    "blue": [
        ((92, 60, 60), (120, 255, 255)),
    ],
    "green": [
        ((42, 60, 60), (85, 255, 255)),
    ],
    "yellow": [
        ((20, 80, 80), (35, 255, 255)),
    ],
    # Pink/magenta fallback. Calibration is still recommended for demo lighting.
    "pink": [
        ((135, 40, 60), (175, 255, 255)),
    ],
}

# BGR drawing colors for OpenCV display.
BOX_COLORS = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "pink": (255, 0, 255),
}


# ============================================================
# Profile loading
# ============================================================
OBJECT_PROFILES = {}


def _as_hsv_tuple(value):
    return tuple(int(x) for x in value)


def load_object_profiles(path=PROFILE_PATH):
    global OBJECT_PROFILES

    if not path.exists():
        print(f"No learned profile found at {path}. Using fallback HSV ranges.")
        OBJECT_PROFILES = {}
        return {}

    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except Exception as exc:
        print(f"Could not load {path}: {exc}. Using fallback HSV ranges.")
        OBJECT_PROFILES = {}
        return {}

    profiles = {}
    for color, profile in raw.items():
        try:
            hsv_ranges = []
            for lower, upper in profile.get("hsv_ranges", []):
                hsv_ranges.append((_as_hsv_tuple(lower), _as_hsv_tuple(upper)))

            if not hsv_ranges:
                print(f"Profile for {color} has no hsv_ranges, skipping.")
                continue

            clean = dict(profile)
            clean["hsv_ranges"] = hsv_ranges

            if "desired_area" in clean:
                DESIRED_AREA[color] = int(clean["desired_area"])

            profiles[color] = clean

        except Exception as exc:
            print(f"Skipping invalid profile for {color}: {exc}")

    OBJECT_PROFILES = profiles

    if OBJECT_PROFILES:
        print("Loaded learned object profiles:")
        for color, profile in OBJECT_PROFILES.items():
            print(
                f"  {color}: desired_area={DESIRED_AREA.get(color)}, "
                f"hsv_ranges={profile.get('hsv_ranges')}, "
                f"aspect={profile.get('aspect_ratio_range')}, "
                f"min_fill={profile.get('min_fill_ratio')}"
            )
    else:
        print("No valid learned profiles found. Using fallback HSV ranges.")

    return OBJECT_PROFILES


def get_hsv_ranges(color_name):
    profile = OBJECT_PROFILES.get(color_name)
    if profile and profile.get("hsv_ranges"):
        return profile["hsv_ranges"]
    return FALLBACK_HSV_RANGES[color_name]


def get_desired_area(color_name):
    return int(DESIRED_AREA.get(color_name, 50000))


# ============================================================
# Motor wrappers
# ============================================================
def stop():
    fc.stop()


def move_forward(power=FORWARD_POWER):
    fc.forward(power)


def turn_left(power=TURN_POWER):
    fc.turn_left(power)


def turn_right(power=TURN_POWER):
    fc.turn_right(power)


# ============================================================
# Manual movement command execution
# ============================================================
def start_manual_sequence(commands):
    """Start a sequence of manual voice movement commands."""
    global _manual_queue, _manual_current, _manual_end_time
    _manual_queue = list(commands)
    _manual_current = None
    _manual_end_time = 0.0
    stop()


def reset_manual_state():
    """Cancel any manual movement."""
    global _manual_queue, _manual_current, _manual_end_time
    _manual_queue = []
    _manual_current = None
    _manual_end_time = 0.0
    stop()


def _manual_duration(command):
    action = command.get("action")
    value = float(command.get("value", 0.0))

    if action == "forward":
        return max(0.0, value * SECONDS_PER_CM)

    if action in {"turn_left", "turn_right"}:
        return max(0.0, value * SECONDS_PER_DEGREE)

    return 0.0


def _start_one_manual_command(command):
    global _manual_current, _manual_end_time

    _manual_current = command
    _manual_end_time = time.time() + _manual_duration(command)

    action = command.get("action")
    if action == "forward":
        move_forward(MANUAL_FORWARD_POWER)
    elif action == "turn_left":
        turn_left(MANUAL_TURN_POWER)
    elif action == "turn_right":
        turn_right(MANUAL_TURN_POWER)
    else:
        stop()


def handle_manual_commands():
    """
    Run manual movement commands sent by the voice pipeline.

    Supported actions:
    - forward N cm
    - turn left N degrees
    - turn right N degrees
    - clockwise turn N degrees, treated as turn right
    """
    global _manual_queue, _manual_current, _manual_end_time

    now = time.time()

    if _manual_current is not None:
        action = _manual_current.get("action", "unknown")
        value = _manual_current.get("value", 0)

        if now < _manual_end_time:
            # Re-issue motor command every loop so the command remains active.
            if action == "forward":
                move_forward(MANUAL_FORWARD_POWER)
            elif action == "turn_left":
                turn_left(MANUAL_TURN_POWER)
            elif action == "turn_right":
                turn_right(MANUAL_TURN_POWER)
            else:
                stop()
            return f"manual {action} {value}"

        stop()
        _manual_current = None
        _manual_end_time = 0.0

    if _manual_queue:
        next_command = _manual_queue.pop(0)
        _start_one_manual_command(next_command)
        action = next_command.get("action", "unknown")
        value = next_command.get("value", 0)
        return f"manual {action} {value}"

    stop()
    return "manual done"


# ============================================================
# Vision
# ============================================================
def build_color_mask(frame_bgr, color_name):
    """
    Build a binary mask for the selected color.

    In this version the captured frame is treated as BGR, so we use BGR -> HSV.
    This should match the previous working behavior where removing RGB->BGR
    conversion fixed the red/blue swap.
    """
    resized = cv2.resize(frame_bgr, (DETECT_W, DETECT_H), interpolation=cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    mask = np.zeros((DETECT_H, DETECT_W), dtype=np.uint8)
    for lower, upper in get_hsv_ranges(color_name):
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        part = cv2.inRange(hsv, lower_np, upper_np)
        mask = cv2.bitwise_or(mask, part)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_5, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_5, iterations=1)
    return mask


def _contour_passes_profile_filters(contour, bbox, color_name, scale_x, scale_y):
    x, y, w, h = bbox
    if w < 8 or h < 8:
        return False

    profile = OBJECT_PROFILES.get(color_name)
    if not profile:
        return True

    full_w = w * scale_x
    full_h = h * scale_y
    full_area = full_w * full_h
    aspect_ratio = h / float(max(w, 1))

    contour_area = cv2.contourArea(contour)
    bbox_area_small = float(max(w * h, 1))
    fill_ratio = contour_area / bbox_area_small

    min_area = float(profile.get("min_area", 0))
    max_area = float(profile.get("max_area", 10**9))
    if full_area < min_area or full_area > max_area:
        return False

    ar_range = profile.get("aspect_ratio_range")
    if ar_range and len(ar_range) == 2:
        ar_min, ar_max = float(ar_range[0]), float(ar_range[1])
        if aspect_ratio < ar_min or aspect_ratio > ar_max:
            return False

    min_fill = float(profile.get("min_fill_ratio", 0.0))
    if fill_ratio < min_fill:
        return False

    return True


def find_largest_color_target(frame_bgr, color_name):
    mask = build_color_mask(frame_bgr, color_name)
    contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_info) == 3:
        _, contours, _ = contours_info
    else:
        contours, _ = contours_info

    scale_x = frame_bgr.shape[1] / float(DETECT_W)
    scale_y = frame_bgr.shape[0] / float(DETECT_H)

    best = None
    best_score = 0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        if not _contour_passes_profile_filters(
            contour, (x, y, w, h), color_name, scale_x, scale_y
        ):
            continue

        full_area = int((w * scale_x) * (h * scale_y))
        if full_area > best_score:
            best_score = full_area
            best = (x, y, w, h, contour)

    if best is None:
        return mask, None

    x, y, w, h, contour = best

    full_x = int(x * scale_x)
    full_y = int(y * scale_y)
    full_w = int(w * scale_x)
    full_h = int(h * scale_y)
    cx = full_x + full_w // 2
    cy = full_y + full_h // 2

    contour_area = cv2.contourArea(contour)
    fill_ratio = contour_area / float(max(w * h, 1))
    aspect_ratio = h / float(max(w, 1))

    return mask, {
        "x": full_x,
        "y": full_y,
        "w": full_w,
        "h": full_h,
        "cx": cx,
        "cy": cy,
        "area": full_w * full_h,
        "aspect_ratio": aspect_ratio,
        "fill_ratio": fill_ratio,
        "profile_used": color_name in OBJECT_PROFILES,
    }


def draw_target(frame_bgr, target, color_name):
    if target is None:
        return

    box_color = BOX_COLORS[color_name]
    x = target["x"]
    y = target["y"]
    w = target["w"]
    h = target["h"]
    cx = target["cx"]
    cy = target["cy"]

    cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), box_color, 2)
    cv2.circle(frame_bgr, (cx, cy), 5, (255, 0, 0), -1)

    profile_flag = "profile" if target.get("profile_used") else "fallback"
    label = (
        f"{color_name} {profile_flag} ({cx},{cy}) "
        f"area={w * h} ar={target.get('aspect_ratio', 0):.2f} "
        f"fill={target.get('fill_ratio', 0):.2f}"
    )

    cv2.putText(
        frame_bgr,
        label,
        (x, max(22, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        box_color,
        2,
    )


# ============================================================
# Control
# ============================================================
def follow_target(target, color_name, frame_width=FRAME_W):
    """
    Normal follow behavior.

    If the target is lost during ordinary following, stop in place.
    The only active search behavior happens after an explicit switch command.
    """
    if target is None:
        stop()
        return "target lost - stopped"

    cx = target["cx"]
    area = target["area"]
    center_x = frame_width // 2
    error_x = cx - center_x

    # Yaw: turn toward the target before moving forward.
    if abs(error_x) > TURN_THRESHOLD:
        if error_x < 0:
            turn_left(TURN_POWER)
            return "turn left"
        turn_right(TURN_POWER)
        return "turn right"

    # Distance zones.
    desired = get_desired_area(color_name)
    far_area = desired * FAR_AREA_RATIO
    slow_area = desired * SLOW_AREA_RATIO
    near_area = desired * NEAR_AREA_RATIO

    if area > near_area:
        stop()
        return "stop close"

    if area < far_area:
        move_forward(FORWARD_POWER)
        return "forward"

    if area < slow_area:
        move_forward(SLOW_FORWARD_POWER)
        return "slow forward"

    stop()
    return "hold"


# ============================================================
# Status overlay
# ============================================================
def draw_status(frame_bgr, receiver, action, target):
    cv2.line(frame_bgr, (FRAME_W // 2, 0), (FRAME_W // 2, FRAME_H), (255, 255, 255), 1)

    color = receiver.current_color
    desired = get_desired_area(color)
    area = target["area"] if target else 0
    source = "learned profile" if color in OBJECT_PROFILES else "fallback HSV"

    lines = [
        f"mode: {receiver.current_mode}",
        f"color: {color} ({source})",
        f"area: {area} / desired {desired}",
        f"last command: {receiver.last_command}",
        f"motion queue: {getattr(receiver, 'motion_commands', [])}",
        f"action: {action}",
    ]

    y = 30
    for line in lines:
        cv2.putText(
            frame_bgr,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 0),
            2,
        )
        y += 28


# ============================================================
# Main
# ============================================================
def main():
    load_object_profiles(PROFILE_PATH)

    receiver = RobotCommandReceiver()
    stop()

    with Picamera2() as camera:
        print("Starting voice-controlled color follower.")
        print("Say: follow red / follow blue / follow green / follow yellow / follow pink / stop.")
        print("Manual: go forward 30 cm / turn left 90 degree / turn right 30 degree / turn clockwise 90 degree.")
        print("Before demo, run calibrate_object.py for your fixed bottle colors.")
        print("Press q or ESC in the OpenCV window to quit.")
        print("Color convention: captured frame is treated as BGR; HSV uses BGR2HSV.")
        print("Switch behavior: after new FOLLOW command, stop for 3s; then scan if target is absent.")
        print(f"FORWARD_POWER={FORWARD_POWER}, SLOW={SLOW_FORWARD_POWER}, "
              f"TURN={TURN_POWER}, SCAN_TURN={SCAN_TURN_POWER}")
        print(f"DESIRED_AREA={DESIRED_AREA}")

        # This can still be RGB888 in Picamera2, but empirically the array is
        # treated as BGR for OpenCV in our setup. The important part is that
        # calibration and runtime use the same conversion.
        camera.preview_configuration.main.size = (FRAME_W, FRAME_H)
        camera.preview_configuration.main.format = "RGB888"
        camera.preview_configuration.align()
        camera.configure("preview")
        camera.start()

        try:
            while True:
                prev_mode = receiver.current_mode
                prev_color = receiver.current_color
                updated = receiver.poll()

                # A new FOLLOW command means the user wants to switch to a new object.
                # Example: laptop sends FOLLOW_YELLOW after "switch to yellow bottle".
                if updated and receiver.current_mode == "follow":
                    reset_manual_state()
                    if prev_mode != "follow" or receiver.current_color != prev_color:
                        start_switch_wait(receiver.current_color)

                # A manual movement command overrides color following.
                if updated and receiver.current_mode == "manual":
                    reset_search_state()
                    start_manual_sequence(receiver.motion_commands)

                frame_bgr = camera.capture_array()
                shown_bgr = frame_bgr.copy()
                target = None

                if receiver.current_mode == "stop":
                    stop()
                    reset_search_state()
                    reset_manual_state()
                    mask = np.zeros((DETECT_H, DETECT_W), dtype=np.uint8)
                    action = "stopped"
                elif receiver.current_mode == "manual":
                    mask = np.zeros((DETECT_H, DETECT_W), dtype=np.uint8)
                    action = handle_manual_commands()
                else:
                    mask, target = find_largest_color_target(
                        frame_bgr,
                        receiver.current_color,
                    )
                    draw_target(shown_bgr, target, receiver.current_color)

                    switch_action = handle_switch_wait_or_scan(
                        target,
                        receiver.current_color,
                        frame_bgr.shape[1],
                    )
                    if switch_action is not None:
                        action = switch_action
                    else:
                        action = follow_target(
                            target,
                            receiver.current_color,
                            frame_bgr.shape[1],
                        )

                draw_status(shown_bgr, receiver, action, target)

                cv2.imshow("voice_color_follow", shown_bgr)
                cv2.imshow("selected_color_mask", mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

                time.sleep(LOOP_SLEEP)

        except KeyboardInterrupt:
            print("Interrupted by user.")
        finally:
            stop()
            receiver.close()
            cv2.destroyAllWindows()
            camera.close()
            print("Quit.")


if __name__ == "__main__":
    main()
