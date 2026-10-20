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
CAMERA_URL = "http://192.168.1.2:8080/video"
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
    (110.0,  110.0),   # P1 front-left
    (230.0,  110.0),   # P2 far-left
    (230.0, -110.0),   # P3 far-right
    (110.0, -110.0),   # P4 front-right
]

# Optional: ArUco markers on the mat corners let the homography be recomputed
# every scan, so a bumped phone does not ruin the calibration.
USE_ARUCO = False
ARUCO_DICT = "DICT_4X4_50"
ARUCO_MARKERS_MM = {          # marker id -> robot-frame XY of the marker CENTRE
    0: (110.0,  110.0),
    1: (230.0,  110.0),
    2: (230.0, -110.0),
    3: (110.0, -110.0),
}

# Pick area in the robot frame. Parts outside this rectangle are ignored
# (keeps the bins, the robot base and table edges out of the search).
WORKSPACE_X = (110.0, 230.0)
WORKSPACE_Y = (-110.0, 110.0)

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
# 4. ARM GEOMETRY  - MEASURE YOUR PRINTED ARM (axis centre to axis centre)
# =============================================================================
# The values below are placeholders in the size range of RoboWings-style 5-DOF arms.
L1 = 95.0      # table surface -> shoulder pitch axis (vertical)
L2 = 105.0     # shoulder axis -> elbow axis
L3 = 100.0     # elbow axis -> wrist pitch axis
L4 = 130.0     # wrist pitch axis -> magnet face, spring at rest =
               # (wrist pitch axis -> top of the printed adapter) + 52 mm (cad/README.md)
SHOULDER_OFFSET = 0.0   # horizontal distance from base yaw axis to shoulder axis (0 on most kits)

# Tool pitch the IK tries, in order. -90 = magnet face straight down (flat contact = full force).
# Flatter angles (-60, -45) extend reach far away; steeper ones (-110, -120) help close in.
PITCH_CANDIDATES = [-90, -85, -95, -80, -100, -75, -105, -70, -110, -65, -120, -60, -55, -50, -45]

# Safety floor: no IK solution may put these points lower than this (mm above table).
# The tool point may go a little below the table: the spring in the magnet housing
# (8 mm travel) absorbs it and keeps the magnet face pressed flat.
MIN_TCP_Z = -4.0
MIN_WRIST_Z = 15.0
MIN_ELBOW_Z = 25.0

# =============================================================================
# 5. SERVO CALIBRATION  (find with arm_console.py)
# =============================================================================
# servo_deg = zero + direction * kinematic_deg
#   zero      : servo angle at which the joint sits at its kinematic 0
#               (base: pointing along +X; shoulder: link horizontal-forward;
#                elbow / wrist pitch: straight in line; roll: any fixed angle, the
#                magnet is round so roll never changes - it stays at kinematic 0)
#   direction : +1 or -1, so that a positive kinematic angle moves the right way
#   min / max : servo-degree limits where nothing hits anything
# Defaults assume the common kit convention "all servos at 90 = arm stands straight up".
JOINTS = {
    "base":        dict(zero=90.0, direction=+1, min=0.0,  max=180.0),
    "shoulder":    dict(zero=0.0,  direction=+1, min=15.0, max=165.0),
    "elbow":       dict(zero=90.0, direction=+1, min=0.0,  max=180.0),
    "wrist_pitch": dict(zero=90.0, direction=+1, min=0.0,  max=180.0),
    "wrist_roll":  dict(zero=90.0, direction=+1, min=0.0,  max=180.0),
}

# Full 6-value poses: 5 servo angles (base, shoulder, elbow, wrist pitch, wrist roll)
# followed by the magnet state (1 = on, 0 = off).
HOME_SERVO = [90, 90, 90, 90, 90, 0]        # must match HOME_POSE in the firmware
PARK_SERVO = [170, 110, 40, 60, 90, 0]      # arm folded OUT of the camera's view

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
    "washer": (70.0,  160.0, 50.0),
    "nut":    (15.0,  175.0, 50.0),
    "bolt":   (70.0, -160.0, 50.0),
}

# =============================================================================
# 7. MOTION
# =============================================================================
APPROACH_HEIGHT = 40.0   # hover this far above the pick / drop point before going straight down
LIFT_HEIGHT = 60.0       # lift this far after the magnet grabs
MOVE_MS = 1200           # free-space moves (firmware may stretch it to respect its speed limits)
DESCEND_MS = 700         # straight down / up near the part
PARK_MS = 1200

# =============================================================================
# 8. SERIAL LINK
# =============================================================================
SERIAL_PORT = "auto"     # "auto" picks the first USB-serial adapter; or "COM5", "/dev/ttyUSB0"
SERIAL_BAUD = 115200
READY_TIMEOUT_S = 4.0    # boards reset when the port opens; wait this long for <READY>
ACK_TIMEOUT_S = 0.5
SEND_RETRIES = 3
DONE_MARGIN_S = 3.0      # extra time allowed beyond the commanded duration

# =============================================================================
# 9. LOGGING
# =============================================================================
RUNS_DIR = HERE / "runs"
