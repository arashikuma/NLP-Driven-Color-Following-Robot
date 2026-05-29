#!/usr/bin/env python3

import socket


UDP_PORT = 9999
VALID_COLORS = {"red", "blue", "green", "yellow", "pink"}


class RobotCommandReceiver:
    """
    Non-blocking UDP receiver for the Raspberry Pi.

    The laptop voice pipeline sends small UDP command strings. The robot polls
    this receiver inside the vision loop and switches mode/color without
    stopping camera processing.
    """

    def __init__(self, host="0.0.0.0", port=UDP_PORT):
        self.current_mode = "stop"
        self.current_color = "red"
        self.motion_commands = []
        self.last_command = "NONE"
        self.last_sender = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        print(f"UDP receiver listening on {host}:{port}")
        print(
            "Commands: "
            + ", ".join(f"FOLLOW_{c.upper()}" for c in sorted(VALID_COLORS))
            + ", STOP"
        )


    def _parse_motion_command(self, command):
        """
        Parse manual motion UDP commands.

        Supported:
        - MOVE_FORWARD_CM_30
        - TURN_LEFT_DEG_90
        - TURN_RIGHT_DEG_30
        - SEQ:TURN_LEFT_DEG_90;MOVE_FORWARD_CM_30
        """
        if command.startswith("SEQ:"):
            parts = [p.strip().upper() for p in command[4:].split(";") if p.strip()]
            parsed = []
            for part in parts:
                one = self._parse_single_motion_command(part)
                if one is None:
                    return None
                parsed.append(one)
            return parsed

        one = self._parse_single_motion_command(command)
        if one is None:
            return None
        return [one]

    def _parse_single_motion_command(self, command):
        if command.startswith("MOVE_FORWARD_CM_"):
            try:
                value = float(command[len("MOVE_FORWARD_CM_"):])
            except ValueError:
                return None
            return {"action": "forward", "value": value, "unit": "cm"}

        if command.startswith("TURN_LEFT_DEG_"):
            try:
                value = float(command[len("TURN_LEFT_DEG_"):])
            except ValueError:
                return None
            return {"action": "turn_left", "value": value, "unit": "deg"}

        if command.startswith("TURN_RIGHT_DEG_"):
            try:
                value = float(command[len("TURN_RIGHT_DEG_"):])
            except ValueError:
                return None
            return {"action": "turn_right", "value": value, "unit": "deg"}

        return None


    def poll(self):
        """
        Read all waiting UDP packets without blocking.

        Returns True if at least one valid command updated the robot state.
        """
        updated = False

        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                break

            command = data.decode("utf-8", errors="ignore").strip().upper()
            self.last_command = command
            self.last_sender = addr

            if command == "STOP":
                self.current_mode = "stop"
                self.motion_commands = []
                updated = True
            elif command.startswith("MOVE_") or command.startswith("TURN_") or command.startswith("SEQ:"):
                parsed_motion = self._parse_motion_command(command)
                if parsed_motion is None:
                    print(f"Ignoring invalid motion command: {command}")
                    continue
                self.current_mode = "manual"
                self.motion_commands = parsed_motion
                updated = True
            elif command.startswith("FOLLOW_"):
                color = command[len("FOLLOW_"):].lower()
                if color in VALID_COLORS:
                    self.current_mode = "follow"
                    self.current_color = color
                    updated = True
                else:
                    print(f"Ignoring unsupported color command: {command}")
                    continue
            else:
                print(f"Ignoring unknown command: {command}")
                continue

            print(
                "Received {cmd} from {addr}; mode={mode}, color={color}, motion={motion}".format(
                    cmd=command,
                    addr=addr,
                    mode=self.current_mode,
                    color=self.current_color,
                    motion=self.motion_commands,
                )
            )

        return updated

    def close(self):
        self.sock.close()


if __name__ == "__main__":
    import time

    receiver = RobotCommandReceiver()

    try:
        while True:
            receiver.poll()
            print(
                f"mode={receiver.current_mode}, "
                f"color={receiver.current_color}, "
                f"motion={receiver.motion_commands}, "
                f"last={receiver.last_command}",
                end="\r",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping UDP receiver.")
    finally:
        receiver.close()
