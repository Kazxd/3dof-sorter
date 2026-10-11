"""
3-DOF Robotic Arm Vision Dashboard  (YOLO edition)
==================================================
Phone (IP Webcam) -> YOLO detection (nut / bolt / washer) -> classical OpenCV refinement -> pixel->mm transform
-> inverse kinematics -> Flask dashboard

Run:   pip install flask opencv-python numpy ultralytics
       python app.py
Open:  http://localhost:5000

If the YOLO weights file is missing, the app automatically falls back to the old
threshold + contour method (classification by area only), so the dashboard still runs.
"""

import json
import math
import os
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, render_template

try:
    from ultralytics import YOLO
except ImportError:  # ultralytics not installed -> classic mode only
    YOLO = None

# =============================================================================
# 1. CALIBRATION / CONFIGURATION  (everything you will want to tweak is here)
# =============================================================================

# --- Video source -------------------------------------------------------------
VIDEO_URL = "http://192.0.0.4:8080/video"   # use 0 for a laptop webcam

# --- YOLO ---------------------------------------------------------------------
# Path to YOUR trained weights with classes such as: nut, bolt, washer.
# (Stock COCO models do NOT know what a nut or bolt is - see train_yolo.py.)
YOLO_WEIGHTS = "best.pt"
YOLO_CONF = 0.50        # minimum confidence (0-1) to accept a detection
YOLO_IMGSZ = 640        # inference size; 416 is faster on weak CPUs
YOLO_DEVICE = None      # None = auto; "cpu", "0" (first GPU), "mps" (Apple)

# --- Class names / colours ----------------------------------------------------
# Your Roboflow dataset names its classes '0'..'5', so the dashboard just showed
# numbers and drew grey boxes. Map each class ID to a real name here.
# Run show_classes.py to see what each number actually is, then edit this dict.
# Any class you leave out is shown as "Class <id>" with an automatic colour.
CLASS_NAMES = {
    # "0": "Washer",
    # "1": "Hex Bolt",
    "2": "Bolt",
    # "3": "Screw",
    "4": "Nut",
    # "5": "Nut",
}
# Optional: force a colour (BGR) for a display name. Otherwise a palette is used.
NAME_COLORS = {"Washer": (255, 200, 0), "Nut": (0, 200, 255), "Bolt": (80, 220, 80)}
# One distinct colour per class ID (BGR), used when a name has no NAME_COLORS entry
PALETTE = [(255, 120, 60), (60, 200, 255), (90, 230, 90), (200, 90, 255),
           (0, 165, 255), (255, 255, 80), (180, 180, 255), (120, 255, 200)]

# --- Pixel -> millimetre scale ------------------------------------------------
# Measure a known length on the paper (e.g. 100 mm), count how many pixels it
# spans in the 640x480 video, then PIXEL_TO_MM = 100 / pixels.
PIXEL_TO_MM = 0.5

# --- Robot base position in the image (fractions of frame size) ---------------
BASE_U_FRAC = 0.5
BASE_V_FRAC = 0.8

# --- Arm geometry (millimetres) - PLACEHOLDERS, measure your arm! -------------
L1 = 60.0    # base height: table surface up to the shoulder pivot axis
L2 = 100.0   # shoulder pivot -> elbow pivot
L3 = 110.0   # elbow pivot -> wrist / gripper tip

# --- Classic (fallback) vision parameters -------------------------------------
BLUR_KERNEL = (7, 7)
THRESHOLD_VALUE = 110    # None = Otsu automatic threshold
MIN_CONTOUR_AREA = 150
MAX_CONTOUR_AREA = 20000
WASHER_MAX_AREA = 600    # fallback classifier: area < 600 -> Washer,
NUT_MAX_AREA = 1500      # < 1500 -> Nut, otherwise Bolt

MAX_OBJECTS_REPORTED = 6

# --- Classical refinement (runs inside every YOLO box) ------------------------
REFINE_PAD = 3            # px of context around each YOLO box (small = fewer neighbours/shadows)
REFINE_MIN_AREA = 60      # ignore blobs smaller than this inside the ROI
MAX_SHIFT_FRAC = 0.20     # reject refinement if centroid moves > 20% of box size from YOLO centre
MIN_FILL, MAX_FILL = 0.12, 1.05   # contour area / box area must lie in this range
SMOOTH_ALPHA = 0.35       # temporal smoothing (0-1, lower = smoother but laggier)
TRACK_MATCH_PX = 40       # max pixel jump to treat a detection as the same object
TRACK_MAX_MISSES = 4      # frames an object may vanish before its track is dropped
MIN_HITS = 2              # a detection must appear in this many frames to be shown
ELONGATED_RATIO = 2.0     # length/width above this -> "elongated" (bolt-like)
ROUND_CIRCULARITY = 0.82  # above this -> "round"
HOLE_MIN_FRAC = 0.03      # hole must be >3% of the blob area to count
EXPECTED_SHAPES = {       # which classical shapes are plausible for each label
    "Bolt":   {"elongated"},
    "Washer": {"round-hole"},
    "Nut":    {"hex-hole", "round-hole"},
}


# =============================================================================
# 2. COORDINATE TRANSFORMATION  (image pixels -> robot base frame, mm)
# =============================================================================
def pixel_to_robot(u, v, frame_w, frame_h):
    """
    Convert image coordinates (u, v) to robot base coordinates (X, Y) in mm.

    Image frame: origin top-left, u -> right, v -> down.
    Robot frame: origin at robot base, X -> right, Y -> forward (up in the image).

        X = (u - cx) * PIXEL_TO_MM
        Y = (cy - v) * PIXEL_TO_MM     (sign flipped because image v points down)

    where (cx, cy) = pixel location of the robot base.
    """
    cx = frame_w * BASE_U_FRAC
    cy = frame_h * BASE_V_FRAC
    return (u - cx) * PIXEL_TO_MM, (cy - v) * PIXEL_TO_MM


# =============================================================================
# 3. INVERSE KINEMATICS  (3-DOF: base pan, shoulder pitch, elbow pitch)
# =============================================================================
def inverse_kinematics(X, Y, Z):
    """
    Return (theta1, theta2, theta3) in degrees, or None if unreachable.

      theta1 : base pan, 0 = straight ahead (+Y), positive toward +X (right)
      theta2 : shoulder pitch from HORIZONTAL (+90 = link 2 pointing up)
      theta3 : elbow angle relative to link 2 (0 = straight, negative = bent
               downward, i.e. the "elbow-up" solution)

    1. Base:      theta1 = atan2(X, Y)
    2. Reduce to a 2-link planar problem:
                  r  = sqrt(X^2 + Y^2)      horizontal reach
                  dz = Z - L1               height of target relative to shoulder
                  d^2 = r^2 + dz^2
    3. Elbow (law of cosines):
                  cos(theta3) = (d^2 - L2^2 - L3^2) / (2*L2*L3)
                  |cos| > 1  ->  out of reach.   theta3 = -acos(...)  (elbow up)
    4. Shoulder:  theta2 = atan2(dz, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3))
    """
    theta1 = math.atan2(X, Y)

    r = math.hypot(X, Y)
    dz = Z - L1
    d2 = r * r + dz * dz

    cos_t3 = (d2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if cos_t3 < -1.0 or cos_t3 > 1.0:
        return None
    theta3 = -math.acos(cos_t3)

    theta2 = math.atan2(dz, r) - math.atan2(L3 * math.sin(theta3),
                                            L2 + L3 * math.cos(theta3))
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)


# =============================================================================
# 4. DETECTORS  (both return the same format: list of dicts)
#    {"label", "u", "v", "box": (x, y, w, h), "conf": float|None}
# =============================================================================
def detect_yolo(model, frame):
    """Run YOLO and convert its boxes into our common detection format."""
    results = model.predict(frame, conf=YOLO_CONF, imgsz=YOLO_IMGSZ,
                            device=YOLO_DEVICE, verbose=False)[0]
    dets = []
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cls_id = int(box.cls[0])
        raw_name = str(model.names[cls_id])
        dets.append({
            "label": CLASS_NAMES.get(raw_name, f"Class {raw_name}"),
            "cls_id": cls_id,
            "u": (x1 + x2) / 2.0,                     # box centre = pick point
            "v": (y1 + y2) / 2.0,
            "box": (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
            "conf": float(box.conf[0]),
        })
    return dets


def classify_by_area(area):
    """Fallback mock classifier (cannot tell nuts from bolts reliably)."""
    if area < WASHER_MAX_AREA:
        return "Washer"
    if area < NUT_MAX_AREA:
        return "Nut"
    return "Bolt"


def detect_classic(frame):
    """Grayscale -> blur -> threshold -> contours (no neural network)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
    if THRESHOLD_VALUE is None:
        _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(blur, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        m = cv2.moments(c)
        u, v = (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] > 0 \
            else (x + bw / 2, y + bh / 2)
        dets.append({"label": classify_by_area(area), "u": u, "v": v,
                     "box": (x, y, bw, bh), "conf": None})
    return dets


# =============================================================================
# 5. SHARED PIPELINE: detections -> mm -> IK -> overlay drawing
# =============================================================================


def refine_classic(frame, det):
    """Classical CV step: refine a YOLO detection using contour analysis in its ROI.
    Updates det in place with a precise centre, orientation, size and shape."""
    H, W = frame.shape[:2]
    x, y, bw, bh = det["box"]
    x0, y0 = max(0, x - REFINE_PAD), max(0, y - REFINE_PAD)
    x1, y1 = min(W, x + bw + REFINE_PAD), min(H, y + bh + REFINE_PAD)
    roi = frame[y0:y1, x0:x1]
    det.update({"refined": False, "angle": None, "length_mm": None, "width_mm": None,
                "shape": None, "holes": None, "circularity": None, "agree": None,
                "_contour": None})
    if roi.size == 0:
        return det

    # 1-4: grayscale -> blur -> Otsu -> morphology
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
    _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Auto polarity: the ROI border is background, so make the border black
    hh, ww = mask.shape
    border = np.concatenate([mask[0, :], mask[-1, :], mask[:, 0], mask[:, -1]])
    centre = mask[hh // 4: 3 * hh // 4, ww // 4: 3 * ww // 4]
    if centre.mean() < border.mean():       # object must be the white blob
        mask = 255 - mask
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # 5: contours with hierarchy (children of an outer contour = holes)
    contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return det
    outer = [i for i, h in enumerate(hier[0])
             if h[3] == -1 and cv2.contourArea(contours[i]) >= REFINE_MIN_AREA]
    if not outer:
        return det
    i = max(outer, key=lambda j: cv2.contourArea(contours[j]))
    c = contours[i]
    area = cv2.contourArea(c)
    per = cv2.arcLength(c, True)

    holes = sum(1 for j, h in enumerate(hier[0])
                if h[3] == i and cv2.contourArea(contours[j]) > HOLE_MIN_FRAC * area)

    # 6: centroid from moments (outer contour is "filled", so holes don't bias it)
    m = cv2.moments(c)
    if m["m00"] == 0:
        return det
    cu, cv_ = m["m10"] / m["m00"] + x0, m["m01"] / m["m00"] + y0

    # Sanity gate: if the contour looks wrong, keep the plain YOLO result
    fill = area / float(max(1, bw * bh))
    shift = math.hypot(cu - det["u"], cv_ - det["v"])
    if not (MIN_FILL <= fill <= MAX_FILL) or shift > MAX_SHIFT_FRAC * max(bw, bh):
        return det

    # 7: oriented rectangle -> angle and real-world size
    (_, _), (rw, rh), ang = cv2.minAreaRect(c)
    length, width = max(rw, rh), max(1e-6, min(rw, rh))
    if rw < rh:
        ang += 90.0
    ang = ang % 180.0                      # long-axis direction, 0..180 deg

    # 8: shape descriptors
    circ = 4 * math.pi * area / (per * per) if per > 0 else 0
    verts = len(cv2.approxPolyDP(c, 0.03 * per, True))
    if length / width > ELONGATED_RATIO:
        shape = "elongated"
    elif holes >= 1 and circ >= ROUND_CIRCULARITY:
        shape = "round-hole"
    elif holes >= 1 and 5 <= verts <= 7:
        shape = "hex-hole"
    else:
        shape = "other"

    # 9: cross-check with the YOLO label
    expected = EXPECTED_SHAPES.get(det["label"])
    agree = (shape in expected) if expected else None

    c_shifted = c + np.array([[x0, y0]])   # back to full-frame coordinates
    det.update({
        "u": cu, "v": cv_, "refined": True,
        "angle": round(ang, 1),
        "length_mm": round(length * PIXEL_TO_MM, 1),
        "width_mm": round(width * PIXEL_TO_MM, 1),
        "shape": shape, "holes": holes, "circularity": round(circ, 2),
        "agree": agree, "_contour": c_shifted,
    })
    return det


class Tracker:
    """Matches detections across frames and smooths them (removes jitter/flicker)."""
    def __init__(self):
        self.tracks = []

    def update(self, dets):
        for t in self.tracks:
            t["matched"] = False
        out = []
        for d in sorted(dets, key=lambda d: -(d["conf"] or 0)):
            best, bd = None, TRACK_MATCH_PX
            for t in self.tracks:
                if t["matched"] or t["label"] != d["label"]:
                    continue
                dist = math.hypot(t["u"] - d["u"], t["v"] - d["v"])
                if dist < bd:
                    best, bd = t, dist
            if best is None:
                best = {"label": d["label"], "u": d["u"], "v": d["v"],
                        "box": [float(b) for b in d["box"]], "ang": None, "hits": 0}
                self.tracks.append(best)
            else:
                a = SMOOTH_ALPHA
                best["u"] += a * (d["u"] - best["u"])
                best["v"] += a * (d["v"] - best["v"])
                best["box"] = [b + a * (n - b) for b, n in zip(best["box"], d["box"])]
            if d.get("angle") is not None:      # angle repeats every 180 deg -> smooth doubled angle
                r = math.radians(2 * d["angle"])
                vec = (math.cos(r), math.sin(r))
                best["ang"] = vec if best["ang"] is None else (
                    best["ang"][0] + SMOOTH_ALPHA * (vec[0] - best["ang"][0]),
                    best["ang"][1] + SMOOTH_ALPHA * (vec[1] - best["ang"][1]))
            best["hits"] += 1
            best["misses"] = 0
            best["matched"] = True
            if best["hits"] >= MIN_HITS:
                nd = dict(d)
                nd["u"], nd["v"] = best["u"], best["v"]
                nd["box"] = tuple(int(round(b)) for b in best["box"])
                if best["ang"] is not None and d.get("angle") is not None:
                    nd["angle"] = round((math.degrees(math.atan2(best["ang"][1], best["ang"][0])) / 2) % 180, 1)
                out.append(nd)
        for t in self.tracks:
            if not t["matched"]:
                t["misses"] = t.get("misses", 0) + 1
        self.tracks = [t for t in self.tracks if t.get("misses", 0) <= TRACK_MAX_MISSES]
        return out


tracker = Tracker()


def process_frame(frame, model):
    h, w = frame.shape[:2]
    dets = detect_yolo(model, frame) if model is not None else detect_classic(frame)
    if model is not None:                       # hybrid: YOLO -> classical refinement
        dets = [refine_classic(frame, d) for d in dets]
    dets = tracker.update(dets)

    # Draw robot base origin so the transform can be verified visually
    base_u, base_v = int(w * BASE_U_FRAC), int(h * BASE_V_FRAC)
    cv2.drawMarker(frame, (base_u, base_v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(frame, "ROBOT BASE (0,0)", (base_u + 10, base_v - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    objects = []
    for d in dets:
        X, Y = pixel_to_robot(d["u"], d["v"], w, h)
        Z = 0.0  # objects lie flat on the paper
        angles = inverse_kinematics(X, Y, Z)
        objects.append({
            "label": d["label"],
            "conf": round(d["conf"], 2) if d["conf"] is not None else None,
            "u": round(d["u"], 1), "v": round(d["v"], 1),
            "X": round(X, 1), "Y": round(Y, 1), "Z": Z,
            "reachable": angles is not None,
            "theta1": round(angles[0], 1) if angles else None,
            "theta2": round(angles[1], 1) if angles else None,
            "theta3": round(angles[2], 1) if angles else None,
            "angle": d.get("angle"), "shape": d.get("shape"),
            "holes": d.get("holes"), "circularity": d.get("circularity"),
            "length_mm": d.get("length_mm"), "width_mm": d.get("width_mm"),
            "refined": d.get("refined", False), "agree": d.get("agree"),
            "_contour": d.get("_contour"),
            "_dist": math.hypot(X, Y), "_box": d["box"], "_cls": d.get("cls_id"),
        })

    # Nearest object to the base first = primary pick target
    objects.sort(key=lambda o: o["_dist"])
    objects = objects[:MAX_OBJECTS_REPORTED]

    for i, o in enumerate(objects):
        x, y, bw, bh = o["_box"]
        color = NAME_COLORS.get(o["label"])
        if color is None:
            cid = o["_cls"] if o["_cls"] is not None else 0
            color = PALETTE[cid % len(PALETTE)]
        if not o["reachable"]:
            color = (0, 0, 255)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 3 if i == 0 else 1)
        if o["_contour"] is not None:           # classical stage, visible in the demo
            cv2.drawContours(frame, [o["_contour"]], -1, (255, 255, 255), 1)
            rect = cv2.minAreaRect(o["_contour"])
            cv2.polylines(frame, [cv2.boxPoints(rect).astype(int)], True, color, 2)
        cv2.circle(frame, (int(o["u"]), int(o["v"])), 4, (0, 0, 255), -1)
        conf = f" {o['conf']:.0%}" if o["conf"] is not None else ""
        extra = f" {o['angle']:.0f}deg" if o["angle"] is not None else ""
        if o["agree"] is False:
            extra += " ?"                        # YOLO and geometry disagree
        tag = f"#{i + 1} {o['label']}{conf}{extra} ({o['X']:.0f},{o['Y']:.0f})mm"
        cv2.putText(frame, tag, (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        if i == 0:
            cv2.line(frame, (base_u, base_v), (int(o["u"]), int(o["v"])), color, 1)

    for o in objects:
        o.pop("_box"); o.pop("_dist"); o.pop("_cls"); o.pop("_contour")
    return frame, objects


# =============================================================================
# 6. BACKGROUND CAPTURE THREAD
# =============================================================================
class VisionWorker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.jpeg = None
        self.objects = []
        self.connected = False
        self.mode = "loading"     # "yolo" or "classic"
        self.model = None

    def load_model(self):
        if YOLO is not None and os.path.exists(YOLO_WEIGHTS):
            self.model = YOLO(YOLO_WEIGHTS)
            self.mode = "yolo"
            print(f"[vision] YOLO loaded: {YOLO_WEIGHTS}  classes={self.model.names}")
        else:
            self.mode = "classic"
            reason = "ultralytics not installed" if YOLO is None else f"{YOLO_WEIGHTS} not found"
            print(f"[vision] {reason} -> using CLASSIC contour mode (area-based labels).")

    def run(self):
        self.load_model()
        while True:
            cap = cv2.VideoCapture(VIDEO_URL)
            if not cap.isOpened():
                self.connected = False
                print(f"[vision] Cannot open {VIDEO_URL}, retrying in 2 s...")
                time.sleep(2)
                continue
            self.connected = True
            print("[vision] Stream connected.")
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                frame = cv2.resize(frame, (640, 480))
                annotated, objects = process_frame(frame, self.model)
                ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self.lock:
                        self.jpeg = buf.tobytes()
                        self.objects = objects
            cap.release()
            self.connected = False
            print("[vision] Stream lost, reconnecting...")
            time.sleep(1)


worker = VisionWorker()

# =============================================================================
# 7. FLASK APP
# =============================================================================
app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


def mjpeg_generator():
    last = None
    while True:
        with worker.lock:
            jpeg = worker.jpeg
        if jpeg is None or jpeg is last:
            time.sleep(0.01)
            continue
        last = jpeg
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"


@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/events")
def events():
    def stream():
        while True:
            with worker.lock:
                payload = {
                    "connected": worker.connected,
                    "mode": worker.mode,
                    "objects": worker.objects,
                    "config": {"pixel_to_mm": PIXEL_TO_MM, "L1": L1, "L2": L2, "L3": L3},
                }
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.1)
    return Response(stream(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


if __name__ == "__main__":
    worker.start()
    app.run(host="0.0.0.0", port=5000, threaded=True)