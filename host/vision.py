"""
vision.py - find washers, nuts and bolts in a top-down frame.

Two detectors run on every frame and are fused:

  YOLO (primary)          trained model, robust to clutter and lighting.
  HSV + contours (backup) colour-threshold the mat, take every non-mat blob,
                          classify it by geometry:
                              long and thin             -> bolt
                              has a hole, round outline -> washer
                              has a hole, hex outline   -> nut

Fusion rules (thresholds in config.py):
  YOLO conf >= CONF_HIGH                  accept YOLO's label
  CONF_LOW <= conf < CONF_HIGH            accept only if the blob's shape agrees
  blob with no accepted YOLO box on it    accept the shape label (source "hsv")
So if YOLO is missing, crashes, or loses confidence, the HSV path still finds parts.

The blob under each accepted box also gives the precise pick point (outline
centroid, better than the box centre) and the long-axis angle used to turn the
wrist so the jaws close across a bolt.
"""
import logging
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

import config as C

log = logging.getLogger("vision")
LABELS = ("washer", "nut", "bolt")
COLORS = {"washer": (255, 200, 0), "nut": (0, 200, 255), "bolt": (80, 220, 80), "unknown": (160, 160, 160)}


@dataclass
class Detection:
    label: str                 # washer / nut / bolt
    u: float                   # pick point, pixels
    v: float
    conf: float                # YOLO confidence, or config.HSV_MIN_SCORE for fallback hits
    source: str                # "yolo", "yolo+shape" or "hsv"
    box: tuple                 # x, y, w, h (pixels)
    angle_px: float = None     # long-axis direction in the image, degrees [0, 180)
    shape: str = None          # what the geometry says, for the overlay
    length_px: float = None
    width_px: float = None
    contour: np.ndarray = None


# =============================================================================
# class names
# =============================================================================
def canonical_label(name):
    """Map a model / dataset class name to washer / nut / bolt, or None."""
    n = str(name).lower()
    hits = [lab for lab, aliases in C.CLASS_ALIASES.items() if any(a in n for a in aliases)]
    return hits[0] if len(hits) == 1 else None


def build_class_map(names):
    """{class_id: name} from the model -> {class_id: label} for the classes we can use."""
    items = names.items() if isinstance(names, dict) else enumerate(names)
    out = {}
    for cid, name in items:
        label = C.CLASS_ID_OVERRIDE.get(int(cid)) or canonical_label(name)
        if label:
            out[int(cid)] = label
        else:
            log.info("model class %s (%r) is not washer/nut/bolt - ignored", cid, name)
    return out


# =============================================================================
# HSV mask + blob geometry
# =============================================================================
def hsv_mask(frame, roi_poly=None):
    """Binary mask (255 = part) from the HSV range in config."""
    k = C.BLUR_KSIZE
    img = cv2.GaussianBlur(frame, (k, k), 0) if k and k > 1 else frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lo, hi = C.HSV_LOWER, C.HSV_UPPER
    if lo[0] <= hi[0]:
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    else:                                     # hue range wraps through red (e.g. 170 -> 10)
        mask = cv2.inRange(hsv, np.array(lo), np.array((179, hi[1], hi[2]))) | \
               cv2.inRange(hsv, np.array((0, lo[1], lo[2])), np.array(hi))
    if C.HSV_MODE == "background":
        mask = cv2.bitwise_not(mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    if roi_poly is not None:
        roi = np.zeros_like(mask)
        cv2.fillPoly(roi, [np.asarray(roi_poly, np.int32)], 255)
        mask = cv2.bitwise_and(mask, roi)
    return mask


def _warp(contour, H, offset):
    """Contour points (pixels, ROI offset added) -> float32 points through homography H."""
    pts = contour.reshape(-1, 1, 2).astype(np.float32) + np.float32(offset)
    return cv2.perspectiveTransform(pts, np.asarray(H, np.float64)).astype(np.float32)


def analyze_blob(outer, holes, offset=(0, 0), metric_H=None):
    """Geometry of one part outline. Returns a dict, or None if degenerate.

    metric_H (pixel -> mm homography, optional): shape numbers are then measured on
    the outline mapped onto the table, so a tilted phone or non-square scaling does
    not turn round washers into "elongated" or "hexagonal" ones."""
    area = cv2.contourArea(outer)
    m = cv2.moments(outer)
    if area <= 0 or m["m00"] == 0:
        return None
    ox, oy = offset
    cu, cv_ = m["m10"] / m["m00"], m["m01"] / m["m00"]

    (_, _), (rw, rh), ang = cv2.minAreaRect(outer)
    length, width = max(rw, rh), max(min(rw, rh), 1e-6)
    if rw < rh:
        ang += 90.0
    ang %= 180.0                                   # long-axis direction in the image

    # shape numbers: on the table plane if we can, otherwise in pixels
    if metric_H is not None:
        s_outer = _warp(outer, metric_H, offset)
        s_holes = [_warp(h, metric_H, offset) for h in holes]
    else:
        s_outer, s_holes = outer.astype(np.float32), [h.astype(np.float32) for h in holes]
    s_area = abs(cv2.contourArea(s_outer))
    sm = cv2.moments(s_outer)
    if s_area <= 0 or sm["m00"] == 0:
        return None
    scx, scy = sm["m10"] / sm["m00"], sm["m01"] / sm["m00"]
    (_, _), (sw, sh), _ = cv2.minAreaRect(s_outer)
    elongation = max(sw, sh) / max(min(sw, sh), 1e-6)
    hole_frac = sum(abs(cv2.contourArea(h)) for h in s_holes) / s_area
    sp = s_outer.reshape(-1, 2).astype(np.float64)
    dist = np.hypot(sp[:, 0] - scx, sp[:, 1] - scy)
    p5, p95 = np.percentile(dist, 5), np.percentile(dist, 95)
    radial_spread = p95 / max(p5, 1e-6) - 1.0      # ~0 for a circle, ~0.15 for a hexagon

    pts = outer.reshape(-1, 2).astype(np.float64)

    if elongation >= C.ELONGATION_BOLT:
        shape = "bolt"
    elif hole_frac >= C.HOLE_MIN_FRAC:
        shape = "nut" if radial_spread >= C.RADIAL_SPREAD_NUT else "washer"
    else:
        shape = "unknown"          # e.g. a nut on its side, a bolt standing up, debris

    pick_u, pick_v = cu, cv_
    if shape == "bolt" and C.BOLT_PICK_SHIFT:
        # Which end is the head? Compare how wide each half of the outline is.
        a = np.radians(ang)
        e = np.array([np.cos(a), np.sin(a)])
        n = np.array([-np.sin(a), np.cos(a)])
        rel = pts - (cu, cv_)
        s, t = rel @ e, rel @ n
        front, back = t[s > 0], t[s < 0]
        if len(front) > 2 and len(back) > 2:
            w_front, w_back = np.ptp(front), np.ptp(back)
            head = 1 if w_front > 1.15 * w_back else (-1 if w_back > 1.15 * w_front else 0)
            pick_u += head * C.BOLT_PICK_SHIFT * length * e[0]
            pick_v += head * C.BOLT_PICK_SHIFT * length * e[1]

    return {
        "area": area, "cu": cu + ox, "cv": cv_ + oy, "pick": (pick_u + ox, pick_v + oy),
        "angle": ang, "length": length, "width": width, "elongation": elongation,
        "hole_frac": hole_frac, "radial_spread": radial_spread, "shape": shape,
        "box": tuple(int(b) for b in cv2.boundingRect(outer)),
        "contour": outer + np.array([[ox, oy]], dtype=outer.dtype),
    }


def find_blobs(mask, metric_H=None):
    """All part-sized outlines in a mask, each analysed. Holes come from the contour hierarchy."""
    contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hier is None:
        return []
    hier = hier[0]
    blobs = []
    for i, c in enumerate(contours):
        if hier[i][3] != -1:                       # this contour is a hole, not an outline
            continue
        area = cv2.contourArea(c)
        if not (C.MIN_BLOB_AREA_PX <= area <= C.MAX_BLOB_AREA_PX):
            continue
        holes, j = [], hier[i][2]                  # walk this outline's children
        while j != -1:
            if cv2.contourArea(contours[j]) >= C.MIN_HOLE_AREA_PX:
                holes.append(contours[j])
            j = hier[j][0]
        info = analyze_blob(c, holes, metric_H=metric_H)
        if info:
            blobs.append(info)
    return blobs


def refine_in_box(frame, box, pad=4, metric_H=None):
    """Otsu threshold inside one YOLO box when the HSV mask has nothing usable there."""
    H, W = frame.shape[:2]
    x, y, w, h = box
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
    roi = frame[y0:y1, x0:x1]
    if roi.size == 0 or min(roi.shape[:2]) < 8:
        return None
    gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    _, m = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    border = np.concatenate([m[0, :], m[-1, :], m[:, 0], m[:, -1]])
    if border.mean() > 127:                        # make the background (box border) black
        m = 255 - m
    contours, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hier is None:
        return None
    hier = hier[0]
    outer = [i for i in range(len(contours)) if hier[i][3] == -1]
    if not outer:
        return None
    i = max(outer, key=lambda k: cv2.contourArea(contours[k]))
    holes = [contours[j] for j in range(len(contours))
             if hier[j][3] == i and cv2.contourArea(contours[j]) >= C.MIN_HOLE_AREA_PX]
    return analyze_blob(contours[i], holes, offset=(x0, y0), metric_H=metric_H)


# =============================================================================
# detector
# =============================================================================
class Detector:
    def __init__(self, weights=None, use_yolo=True):
        self.model = None
        self.class_map = {}
        self._yolo_failed = False
        if use_yolo:
            self._load_yolo(Path(weights or C.YOLO_WEIGHTS))
        self.mode = "yolo+hsv" if self.model else "hsv-only"

    def _load_yolo(self, weights):
        if not weights.exists():
            log.warning("no YOLO weights at %s - running HSV fallback only (see get_model.py)", weights)
            return
        try:
            from ultralytics import YOLO
        except ImportError:
            log.warning("ultralytics not installed - running HSV fallback only")
            return
        try:
            model = YOLO(str(weights))
        except Exception as e:                    # corrupt file, wrong version, ...
            log.error("could not load %s: %s - running HSV fallback only", weights, e)
            return
        self.class_map = build_class_map(model.names)
        if not self.class_map:
            log.error("none of the model's classes %s map to washer/nut/bolt; "
                      "set CLASS_ID_OVERRIDE in config.py", model.names)
            return
        self.model = model
        log.info("YOLO loaded: %s  classes used: %s", weights.name, self.class_map)

    def _yolo(self, frame):
        try:
            res = self.model.predict(frame, conf=C.CONF_LOW, imgsz=C.YOLO_IMGSZ,
                                     device=C.YOLO_DEVICE, verbose=False)[0]
        except Exception as e:
            if not self._yolo_failed:
                log.error("YOLO inference failed (%s) - using HSV fallback", e)
                self._yolo_failed = True
            return []
        self._yolo_failed = False
        if res.boxes is None or len(res.boxes) == 0:
            return []
        xyxy = res.boxes.xyxy.cpu().numpy()
        cls = res.boxes.cls.cpu().numpy().astype(int)
        conf = res.boxes.conf.cpu().numpy()
        out = []
        for (x1, y1, x2, y2), c, p in zip(xyxy, cls, conf):
            label = self.class_map.get(int(c))
            if label:
                out.append({"label": label, "conf": float(p),
                            "box": (int(x1), int(y1), int(x2 - x1), int(y2 - y1))})
        return out

    def detect(self, frame, roi_poly=None, metric_H=None):
        """Detections in one frame. roi_poly (image polygon) limits where to look;
        metric_H (pixel -> mm homography) makes the shape rules tilt-proof."""
        mask = hsv_mask(frame, roi_poly)
        blobs = find_blobs(mask, metric_H)
        raw = self._yolo(frame) if self.model is not None else []

        dets, used = [], set()
        for y in sorted(raw, key=lambda d: -d["conf"]):
            x, yy, w, h = y["box"]
            cx, cy = x + w / 2.0, yy + h / 2.0
            if roi_poly is not None and cv2.pointPolygonTest(
                    np.asarray(roi_poly, np.float32), (float(cx), float(cy)), False) < 0:
                continue
            # the blob that belongs to this box: centroid inside, not a merged clump
            inside = [i for i, b in enumerate(blobs) if i not in used
                      and x <= b["cu"] <= x + w and yy <= b["cv"] <= yy + h
                      and b["area"] <= 1.8 * w * h]
            bi = max(inside, key=lambda i: blobs[i]["area"]) if inside else None
            geo = blobs[bi] if bi is not None else refine_in_box(frame, y["box"], metric_H=metric_H)
            shape = geo["shape"] if geo else None

            if y["conf"] >= C.CONF_HIGH:
                source = "yolo"
            elif shape == y["label"]:
                source = "yolo+shape"
            else:
                continue                       # unsure YOLO box: let the HSV path decide
            if bi is not None:
                used.add(bi)
            pu, pv = geo["pick"] if geo else (cx, cy)
            dets.append(Detection(
                label=y["label"], u=pu, v=pv, conf=y["conf"], source=source, box=y["box"],
                angle_px=geo["angle"] if geo else None, shape=shape,
                length_px=geo["length"] if geo else None, width_px=geo["width"] if geo else None,
                contour=geo["contour"] if geo else None))

        for i, b in enumerate(blobs):
            if i in used or b["shape"] == "unknown":
                continue
            if any(d.box[0] <= b["cu"] <= d.box[0] + d.box[2] and
                   d.box[1] <= b["cv"] <= d.box[1] + d.box[3] for d in dets):
                continue                       # already covered by a YOLO detection
            dets.append(Detection(
                label=b["shape"], u=b["pick"][0], v=b["pick"][1], conf=C.HSV_MIN_SCORE,
                source="hsv", box=b["box"], angle_px=b["angle"], shape=b["shape"],
                length_px=b["length"], width_px=b["width"], contour=b["contour"]))
        return dets, mask


# =============================================================================
# drawing
# =============================================================================
def draw_detection(img, d, text=None, thick=1):
    color = COLORS.get(d.label, COLORS["unknown"])
    x, y, w, h = d.box
    cv2.rectangle(img, (x, y), (x + w, y + h), color, thick)
    if d.contour is not None:
        cv2.drawContours(img, [d.contour], -1, (255, 255, 255), 1)
    cv2.circle(img, (int(d.u), int(d.v)), 4, (0, 0, 255), -1)
    if d.angle_px is not None and d.length_px:
        a = np.radians(d.angle_px)
        dx, dy = np.cos(a) * d.length_px / 2, np.sin(a) * d.length_px / 2
        cv2.line(img, (int(d.u - dx), int(d.v - dy)), (int(d.u + dx), int(d.v + dy)), color, 1)
    label = text or f"{d.label} {d.conf:.2f} {d.source}"
    cv2.putText(img, label, (x, max(14, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
