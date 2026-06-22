HOME_SERVO = [89, 91, 91, 90, 89, 179]
2
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
5
SAFE_X_MIN = 60.0
SAFE_X_MAX = 170.0
SAFE_Y_MIN = -100.0
SAFE_Y_MAX = 100.0
SAFE_Z_MIN = 130.0
SAFE_Z_MAX = 170.0
2
DEFAULT_ROLL_DEG = 0.0
DEFAULT_GRIP = 1.0
5
# Measured geometry
L1 = 90.0
L2 = 83.0
L3 = 83.0
L4 = 79.6
L5 = 102.7
2
# Safe hover effective wrist
USE_EFFECTIVE_WRIST = True
EFFECTIVE_L45 = 40.0
6
# Joint limits for reduced hover IK
Q1_MIN = -135.0
Q1_MAX = 135.0
0
Q2_MIN = -40.0
Q2_MAX = 100.0
3
Q3_MIN = -135.0
Q3_MAX = 135.0
6
Q4_MIN = -160.0
Q4_MAX = 160.0
9
# Servo mapping flags
INVERT_BASE = False
INVERT_SHOULDER = True
INVERT_ELBOW = True
INVERT_WRIST_PITCH = True
INVERT_WRIST_ROLL = False
6
# Servo ranges
SERVO_MIN = 0
SERVO_MAX = 180
0
# Motion
MOVE_TIME_MS = 1200
STARTUP_HOME = True
DRY_RUN = False
VERBOSE = True
6
7
# Pick and place sequence
SEQUENCE_MODE = False
0
APPROACH_Z = 112.0
PICK_Z = 105.0
LIFT_Z = 118.0
4
PICK_PITCH = -50.0
PLACE_PITCH = -50.0
7
GRIP_OPEN = 1.0
GRIP_CLOSE = 0.0
0
STEP_MOVE_TIME_MS = 1200
STEP_WAIT_SEC = 1.6
2
3
# Fixed place box for red object
PLACE_X = 120.0
PLACE_Y = -40.0
PLACE_Z = 112.0
8
FIX_WRIST_PITCH = True
FIXED_WRIST_PITCH_SERVO = 15
1
# – TCP / tool-offset correction (added for pick calibration) –––––––
# True physical link lengths
L4_REAL = 79.6
L5_REAL = 102.7
6
# Fixed wrist pitch at hover/pick (degrees, negative = tip down)
WRIST_PITCH_DEG = -62.0
9
