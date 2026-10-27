"""
config.py - every number you measure or tune lives here.

Units: millimetres, degrees, seconds (unless the name says _MS / _PX).

Robot frame used everywhere on the host:
    origin  = the point on the table directly under the base rotation axis
    +X      = straight out in front of the robot
    +Y      = to the robot's left (seen from behind the robot)
    +Z      = up

Work through README.md "Calibration checklist" top to bottom; each step tells you
which block below to edit.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent

# =============================================================================
# 1. CAMERA
# =============================================================================
# IP Webcam (Android) MJPEG URL, an RTSP URL, a webcam index like 0,
# or a path to a still image / video file for offline testing.
CAMERA_URL = "http://192.168.1.3:8080/video"
CAMERA_RECONNECT_S = 2.0      # wait between reconnect attempts
CAMERA_STALE_S = 2.0          # no new frame for this long = camera counts as down

# =============================================================================
# 2. CAMERA -> TABLE MAPPING (homography)
# =============================================================================
CALIB_DIR = HERE / "calibration"
HOMOGRAPHY_FILE = CALIB_DIR / "homography.json"

# Robot-frame (X, Y) in mm of the reference points you click in calibrate_camera.py,
# in the order you will click them. Mark them on the mat (printed crosses or
# ArUco markers) and MEASURE them from the base axis with a ruler.
# 4 points is the minimum; 6-9 points spread over the mat average out click error.
CALIB_POINTS_MM = [
    (140.0,   60.0),   # P1 near-left   (demo rectangle around the pick area:
    (220.0,   60.0),   # P2 far-left     tape crosses at these spots, measured
    (220.0,  -60.0),   # P3 far-right    from the point under the base axis)
    (140.0,  -60.0),   # P4 near-right
]

# Optional: ArUco markers on the mat corners let the homography be recomputed
# every scan, so a bumped phone does not ruin the calibration.
USE_ARUCO = False
ARUCO_DICT = "DICT_4X4_50"
ARUCO_MARKERS_MM = {          # marker id -> robot-frame XY of the marker CENTRE
    0: (140.0,  60.0),
    1: (220.0,  60.0),
    2: (220.0, -60.0),
    3: (140.0, -60.0),
}

# Pick area in the robot frame. Parts outside this rectangle are ignored
# (keeps the bins, the robot base and table edges out of the search).
WORKSPACE_X = (163.0, 186.0)  # the hanging magnet reaches the table ~160-192 mm out
WORKSPACE_Y = (-30.0, 30.0)

# =============================================================================
# 3. VISION
# =============================================================================
YOLO_WEIGHTS = HERE / "models" / "best.pt"   # produced by get_model.py
YOLO_IMGSZ = 640              # 416 if the CPU is slow
YOLO_DEVICE = None            # None = auto, "cpu", "0" = first CUDA GPU

# Two-level confidence:
#   conf >= CONF_HIGH            -> accepted on YOLO alone
#   CONF_LOW <= conf < CONF_HIGH -> accepted only if the contour shape check agrees
#   below CONF_LOW / not found   -> the HSV + contour fallback decides
CONF_HIGH = 0.60
CONF_LOW = 0.30

# Model class names are matched to our three labels by substring (case-insensitive).
# A name matching more than one label (e.g. "bolt-nut-washer") is ignored.
CLASS_ALIASES = {
    "washer": ("washer",),
    "nut":    ("nut",),
    "bolt":   ("bolt", "screw"),
}
# For datasets that export bare numbers as class names, map id -> label here,
# e.g. {2: "bolt", 4: "nut"}. Takes priority over CLASS_ALIASES.
CLASS_ID_OVERRIDE = {}

# --- HSV fallback (tune live with hsv_tuner.py) --------------------------------
# "background": pixels INSIDE the HSV range are the mat, everything else is a part.
#               Best with a coloured mat (green cutting mat) under silver parts.
# "object":     pixels inside the range are parts.
HSV_MODE = "background"
HSV_LOWER = (35, 60, 40)      # (H 0-179, S 0-255, V 0-255) - green cutting mat
HSV_UPPER = (85, 255, 255)    # if H lower > H upper the range wraps through red
BLUR_KSIZE = 5                # odd number; 0 disables blur
MIN_BLOB_AREA_PX = 120
MAX_BLOB_AREA_PX = 40000
MIN_HOLE_AREA_PX = 12

# Shape rules used by the fallback and by the YOLO cross-check
ELONGATION_BOLT = 2.2         # long side / short side above this -> bolt
HOLE_MIN_FRAC = 0.04          # hole area / outline area above this -> has a hole
RADIAL_SPREAD_NUT = 0.09      # (95th / 5th percentile of centroid->edge distance) - 1,
                              # measured in mm on the table: washer ~0.02-0.06, nut ~0.12-0.15
HSV_MIN_SCORE = 0.5           # fallback detections get conf = this (they have no real score)

# Bolt pick point: shift from the silhouette centroid towards the head by this
# fraction of the bolt length, so the magnet lifts near the centre of mass. 0 = centroid.
BOLT_PICK_SHIFT = 0.10

# --- Scan / consensus ----------------------------------------------------------
SCAN_FRAMES = 5               # frames averaged per scan (arm parked, scene still)
SCAN_SETTLE_S = 0.6           # wait after the arm parks before trusting frames
CLUSTER_MM = 8.0              # detections closer than this across frames = same part
MIN_FRAME_FRAC = 0.6          # part must appear in >= 60% of the scan frames
ISOLATION_MM = 25.0           # neighbours closer than this make a pick risky (pushed to the back)
VERIFY_RADIUS_MM = 12.0       # same-label part still within this after a pick = pick failed
MAX_ATTEMPTS = 2              # failed picks at one spot before it is skipped for the session
IDLE_WAIT_S = 1.5             # pause between scans when nothing is pickable

# =============================================================================
# 4. ARM GEOMETRY -- provisional measurements for the user's 3-DOF arm
# =============================================================================
# Units are mm. L1: table to shoulder rotation axis; L2: shoulder-to-elbow axes.
# L3: elbow axis to the hanger's pivot pegs (MEASURE: 75 is read off a photo).
L1 = 95.0       # 62 mm table -> top of the blue turntable + 33 mm -> shoulder screw (measured)
L2 = 120.0
L3 = 75.0

# The magnet swings freely on the printed hanger (cad/magnet_hanger.scad), so it always
# hangs face-down: its face is TOOL_DOWN straight below the pivot, whatever the forearm does.
# "rigid" is the older model (magnet fixed to the forearm, L3 to the face, pitch follows it).
TOOL_MOUNT = "hanging"
TOOL_FORWARD = 0.0
TOOL_DOWN = 61.0         # hanger pivot -> magnet face (PETG hanger)
TOOL_AXIS_DEG = -90.0    # rigid mode only: face-down relative to a horizontal forearm
TOOL_SELF_LEVELING = True
HANG_MAX_FOREARM_UP_DEG = 30.0   # steeper and the hanger rests on the forearm
SHOULDER_OFFSET = 0.0

# Preferred absolute tool pitch. For this 3-DOF arm this is a preference/check,
# not a command to a nonexistent wrist-pitch servo.
PITCH_CANDIDATES = [-90]
TOOL_PITCH_TOLERANCE_DEG = 25.0  # initial limit; validate contact empirically

# Safety floor checks (mm above table).
MIN_TCP_Z = -4.0
MIN_WRIST_Z = 15.0
MIN_ELBOW_Z = 25.0

# =============================================================================
# 5. SERVO CALIBRATION -- provisional, verify each limit mechanically
# =============================================================================
# From the stated observations: 90 deg is the base-forward / shoulder-horizontal /
# elbow-straight pose. Increasing all three servo commands moves in the positive
# kinematic direction. 60..120 deg is only the current conservative test window.
# W4/W5 are not actuated by the adapted firmware; their command slots stay at 90.
JOINTS = {
    "base":        dict(zero=90.0, direction=+1, min=5.0, max=175.0),
    "shoulder":    dict(zero=90.0, direction=+1, min=10.0, max=175.0),  # 180 = just past vertical
    "elbow":       dict(zero=90.0, direction=-1, min=10.0, max=160.0),  # bigger servo deg folds the forearm DOWN
    "wrist_pitch": dict(zero=90.0, direction=+1, min=10.0, max=160.0),
    "wrist_roll":  dict(zero=90.0, direction=+1, min=10.0, max=160.0),
}

# Protocol still contains 5 servo angles + magnet state. Keep both poses six-wide.
# Upright (shoulder 170 ~ vertical, elbow straight): centre of mass over the base,
# so the arm starts and rests without tipping. Must match HOME_POSE in the firmware.
HOME_SERVO = [90, 170, 90, 90, 90, 0]
# Bench-safe placeholder only. After collision-checking, choose a PARK pose that
# moves the arm out of the camera view and remains inside the calibrated limits.
PARK_SERVO = [90, 170, 90, 90, 90, 0]

# =============================================================================
# 6. ELECTROMAGNET, PICK HEIGHTS AND BINS
# =============================================================================
# End effector: KK-P20/15 lifting electromagnet, 12 V DC, about 0.2 A, 20 mm x 15 mm,
# up to 3 kg on flat clean mild steel. It sits in the spring-loaded printed housing
# in cad/ and is switched by the Arduino through a logic-level MOSFET (README wiring).
# Only ferromagnetic parts lift: plain or zinc-plated steel. Stainless A2/A4 and brass will not.
#
# Per class: z = height of the part's top face above the table, which is where the
# magnet face has to touch. Measure your parts with calipers.
PICK = {
    "washer": dict(z=1.6),     # M6 washer thickness
    "nut":    dict(z=5.0),     # M6 nut height
    "bolt":   dict(z=6.0),     # M6 bolt lying flat: shank diameter
}
PRESS_DEPTH = 3.0          # go this far below the part's top; the housing spring takes it up
                           # so the face always seats flat, even with a few mm of arm error
MAGNET_SETTLE_MS = 250     # magnet on, wait before lifting (coil current takes time to build)
RELEASE_DWELL_MS = 400     # magnet off over the bin, wait before moving (residual magnetism)

# Bin drop points in the robot frame: (X, Y, Z of the tool point when releasing).
# Put the bins outside WORKSPACE_* so they are never detected as parts.
BINS = {
    "washer": (102.0,  146.0, 50.0),   # 55 deg left of forward
    "nut":    (102.0, -146.0, 50.0),   # 55 deg right
    "bolt":   ( 31.0,  175.0, 50.0),   # 80 deg left
}  # provisional: the hanging magnet reaches the table only ~160-192 mm from the base axis,
   # so every bin sits on a ~178 mm radius, beside the pick area

# =============================================================================
# 7. MOTION
# =============================================================================
APPROACH_HEIGHT = 15.0   # conservative for the current three-joint test workspace   # hover this far above the pick / drop point before going straight down
LIFT_HEIGHT = 15.0       # increase only after a staged reachability test       # lift this far after the magnet grabs
MOVE_MS = 1200           # free-space moves (firmware may stretch it to respect its speed limits)
DESCEND_MS = 700         # straight down / up near the part
PARK_MS = 1200

# =============================================================================
# 8. SERIAL LINK
# =============================================================================
SERIAL_PORT = "tcp://192.168.4.1:8888"  # ESP32 Wi-Fi TCP
SERIAL_BAUD = 115200
READY_TIMEOUT_S = 4.0    # boards reset when the port opens; wait this long for <READY>
ACK_TIMEOUT_S = 0.5
SEND_RETRIES = 3
DONE_MARGIN_S = 3.0      # extra time allowed beyond the commanded duration

# =============================================================================
# 9. LOGGING
# =============================================================================
RUNS_DIR = HERE / "runs"
