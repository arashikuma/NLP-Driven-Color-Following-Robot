# Robot voice manual movement commands

Added only these English voice movement commands:
- go forward N cm
- go straight N cm
- turn left N degree
- turn right N degree
- turn clockwise N degree
- sequences joined by "and", e.g. turn left 90 degree and go straight 30 cm

The laptop sends:
- MOVE_FORWARD_CM_N
- TURN_LEFT_DEG_N
- TURN_RIGHT_DEG_N
- SEQ:TURN_LEFT_DEG_90;MOVE_FORWARD_CM_30

The Pi receives manual commands and executes them with time-based calibration constants:
- SECONDS_PER_CM
- SECONDS_PER_DEGREE

Tune those two constants in robot_color_follow.py if the distance/angle is inaccurate.
