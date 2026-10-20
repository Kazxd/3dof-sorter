"""
synthetic.py - draw a fake top-down scene with parts at known millimetre positions.

Used by the tests, and handy for trying the whole pipeline without a camera:

    python tests/synthetic.py           writes tests/out/scene.png + calibration
    python sorter.py --camera tests/out/scene.png --sim --no-yolo
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parent / "out"
SIZE = (960, 720)                               # width, height (4:3 like a phone stream)

# mm -> px at ~3 px/mm with a little keystone, as if the phone were tilted a few degrees
# (the far edge of the mat looks narrower than the near edge)
MM_PTS = np.float32([[110, 110], [230, 110], [230, -110], [110, -110]])
PX_PTS = np.float32([[150, 610], [172, 250], [792, 245], [812, 606]])
H_MM2PX = cv2.getPerspectiveTransform(MM_PTS, PX_PTS)
H_PX2MM = np.linalg.inv(H_MM2PX)

# label, x, y, long-axis angle (deg, robot frame)
PARTS = [
    ("washer", 150.0, 40.0, 0.0),
    ("nut", 185.0, -55.0, 15.0),
    ("bolt", 205.0, 35.0, 30.0),
    ("nut", 135.0, -20.0, 0.0),
]


def _to_px(pts_mm):
    p = np.asarray(pts_mm, np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, H_MM2PX).reshape(-1, 2).astype(np.int32)


def _circle(cx, cy, r, n=90):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _poly(cx, cy, r, sides, rot_deg):
    a = np.radians(rot_deg) + np.linspace(0, 2 * math.pi, sides, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], axis=1)


def _rect(cx, cy, length, width, ang_deg):
    a = math.radians(ang_deg)
    e, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    c = np.array([cx, cy])
    return np.array([c - e * length / 2 - n * width / 2, c + e * length / 2 - n * width / 2,
                     c + e * length / 2 + n * width / 2, c - e * length / 2 + n * width / 2])


def make_scene(parts=PARTS, seed=0):
    rng = np.random.default_rng(seed)
    w, h = SIZE
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (60, 140, 60)                      # green cutting mat (BGR)
    metal = (175, 175, 170)
    for label, x, y, ang in parts:
        if label == "washer":
            cv2.fillPoly(img, [_to_px(_circle(x, y, 6.0))], metal)
            cv2.fillPoly(img, [_to_px(_circle(x, y, 3.2))], (60, 140, 60))
        elif label == "nut":
            cv2.fillPoly(img, [_to_px(_poly(x, y, 5.77, 6, ang))], metal)
            cv2.fillPoly(img, [_to_px(_circle(x, y, 2.5))], (60, 140, 60))
        elif label == "bolt":
            a = math.radians(ang)
            # shank 30 x 6 mm, hex head 4 mm long x 10 mm wide at the +axis end
            cv2.fillPoly(img, [_to_px(_rect(x, y, 30.0, 6.0, ang))], metal)
            hx, hy = x + 15.0 * math.cos(a), y + 15.0 * math.sin(a)
            cv2.fillPoly(img, [_to_px(_rect(hx, hy, 4.0, 10.0, ang))], metal)
    noise = rng.normal(0, 4, img.shape)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def write_files(out_dir=OUT):
    out_dir.mkdir(parents=True, exist_ok=True)
    scene = out_dir / "scene.png"
    cv2.imwrite(str(scene), make_scene())
    calib = out_dir / "homography.json"
    calib.write_text(json.dumps({"H": H_PX2MM.tolist(), "image_size": list(SIZE), "rms_mm": 0.0}),
                     encoding="utf-8")
    return scene, calib


if __name__ == "__main__":
    s, c = write_files()
    print(f"wrote {s} and {c}\nCopy the json to calibration/homography.json to run sorter.py on it.")
