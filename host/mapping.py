"""
mapping.py - image pixels <-> table millimetres (robot frame) with a homography.

Why a homography: the camera looks down at a flat table. Any flat plane seen
through a pinhole camera maps to the image by a 3x3 projective matrix H, even if
the phone is a little tilted. Four or more known points on the table fix H.
A single "mm per pixel" factor only works if the phone is perfectly level.

Parallax note: H is exact for points ON the table. The top of a 5 mm nut seen
from 500 mm up is displaced by ~1% of its distance from the image centre,
which is well under a millimetre here, so it is ignored.
"""
import json
import math
import time

import cv2
import numpy as np

import config as C


class PlaneMapper:
    def __init__(self, H, image_size, rms_mm=None):
        self.H = np.asarray(H, dtype=np.float64)
        self.H_inv = np.linalg.inv(self.H)
        self.image_size = (int(image_size[0]), int(image_size[1]))   # (width, height)
        self.rms_mm = rms_mm
        self._scale = np.eye(3)        # adapts H if the stream resolution changes

    # ---------------------------------------------------------------- building
    @classmethod
    def from_points(cls, px_points, mm_points, image_size):
        """Fit H from >= 4 pixel/mm correspondences. RANSAC kicks in with > 4 points."""
        px = np.asarray(px_points, dtype=np.float64).reshape(-1, 1, 2)
        mm = np.asarray(mm_points, dtype=np.float64).reshape(-1, 1, 2)
        if len(px) < 4 or len(px) != len(mm):
            raise ValueError("need at least 4 matching pixel and mm points")
        method = cv2.RANSAC if len(px) > 4 else 0
        H, _ = cv2.findHomography(px, mm, method, 3.0)
        if H is None:
            raise ValueError("homography failed - are three of the points on one line?")
        mapper = cls(H, image_size)
        proj = cv2.perspectiveTransform(px, mapper.H).reshape(-1, 2)
        mapper.rms_mm = float(np.sqrt(np.mean(np.sum((proj - mm.reshape(-1, 2)) ** 2, axis=1))))
        return mapper

    @classmethod
    def load(cls, path=None):
        path = path or C.HOMOGRAPHY_FILE
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        return cls(d["H"], d["image_size"], d.get("rms_mm"))

    def save(self, path=None, extra=None):
        path = path or C.HOMOGRAPHY_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {"H": self.H.tolist(), "image_size": list(self.image_size),
                "rms_mm": self.rms_mm, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        if extra:
            data.update(extra)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    # ------------------------------------------------------- resolution guard
    def set_frame_size(self, width, height):
        """Call once per frame size. Rescales pixels if the stream resolution differs
        from the calibration image, and warns if the aspect ratio changed (that
        would mean the image is cropped or stretched and H is no longer valid)."""
        cw, ch = self.image_size
        if (width, height) == (cw, ch):
            self._scale = np.eye(3)
            return True
        sx, sy = cw / float(width), ch / float(height)
        self._scale = np.diag([sx, sy, 1.0])
        return abs(sx - sy) / max(sx, sy) < 0.02     # False = aspect changed, recalibrate

    # ---------------------------------------------------------------- mapping
    def matrix(self):
        """Pixel -> mm homography for the current frame size."""
        return self.H @ self._scale

    def px_to_mm(self, u, v):
        p = self.H @ self._scale @ np.array([u, v, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def mm_to_px(self, x, y):
        p = np.linalg.inv(self._scale) @ self.H_inv @ np.array([x, y, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def axis_px_to_mm(self, u, v, angle_px_deg, step_px=10.0):
        """Direction of a line through (u, v) at angle_px (image degrees) -> robot-frame
        degrees in [0, 180). Done by mapping two points, so mirrored or rotated
        camera mounts are handled automatically."""
        a = math.radians(angle_px_deg)
        du, dv = math.cos(a) * step_px, math.sin(a) * step_px
        x0, y0 = self.px_to_mm(u - du, v - dv)
        x1, y1 = self.px_to_mm(u + du, v + dv)
        return math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0

    def workspace_polygon_px(self, margin_mm=0.0):
        """Pick-area rectangle (config.WORKSPACE_*), grown by margin_mm, as an int32 image polygon.
        Vision searches a slightly larger area so parts on the edge are not cut in half."""
        x0, x1 = C.WORKSPACE_X[0] - margin_mm, C.WORKSPACE_X[1] + margin_mm
        y0, y1 = C.WORKSPACE_Y[0] - margin_mm, C.WORKSPACE_Y[1] + margin_mm
        pts = [self.mm_to_px(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        return np.array(pts, dtype=np.int32)


def in_workspace(x, y):
    return C.WORKSPACE_X[0] <= x <= C.WORKSPACE_X[1] and C.WORKSPACE_Y[0] <= y <= C.WORKSPACE_Y[1]


# =============================================================================
# ArUco helpers (optional live recalibration)
# =============================================================================
def detect_aruco_centres(frame, dict_name=None):
    """Return {marker_id: (u, v)} of marker centres, or {} if ArUco is unavailable."""
    if not hasattr(cv2, "aruco"):
        return {}
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name or C.ARUCO_DICT))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    if hasattr(cv2.aruco, "ArucoDetector"):                      # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:                                                        # older OpenCV
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return {}
    return {int(i): tuple(c.reshape(4, 2).mean(axis=0)) for i, c in zip(ids.flatten(), corners)}


def mapper_from_aruco(frame):
    """Fit a PlaneMapper from the markers in config.ARUCO_MARKERS_MM, or None if < 4 are visible."""
    centres = detect_aruco_centres(frame)
    ids = [i for i in C.ARUCO_MARKERS_MM if i in centres]
    if len(ids) < 4:
        return None
    h, w = frame.shape[:2]
    return PlaneMapper.from_points([centres[i] for i in ids],
                                   [C.ARUCO_MARKERS_MM[i] for i in ids], (w, h))
