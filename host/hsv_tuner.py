"""
hsv_tuner.py - tune the HSV fallback live and watch how it classifies parts.

    python hsv_tuner.py                 camera from config
    python hsv_tuner.py --camera mat.jpg

Drag the sliders until the mask (right) is black on the mat and white on every
part, with washer and nut holes showing as black dots. Labels on the left show
what the shape rules decide (bolt / washer / nut / unknown) plus the measured
numbers, so you can adjust ELONGATION_BOLT, HOLE_MIN_FRAC and RADIAL_SPREAD_NUT.

p = print the config lines to paste into config.py      q / Esc = quit
"""
import argparse

import cv2
import numpy as np

import config as C
import vision
from camera import FrameGrabber
from mapping import PlaneMapper

WIN = "hsv tuner"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera")
    args = ap.parse_args()
    grab = FrameGrabber(args.camera)
    grab.start()

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    lo, hi = C.HSV_LOWER, C.HSV_UPPER
    for name, val, top in (("H low", lo[0], 179), ("H high", hi[0], 179), ("S low", lo[1], 255),
                           ("S high", hi[1], 255), ("V low", lo[2], 255), ("V high", hi[2], 255)):
        cv2.createTrackbar(name, WIN, int(val), top, lambda _v: None)
    cv2.createTrackbar("mode: 0 bg / 1 obj", WIN, 0 if C.HSV_MODE == "background" else 1, 1, lambda _v: None)
    # with a saved calibration the shape numbers are measured in mm, exactly like sorter.py
    mapper = PlaneMapper.load() if C.HOMOGRAPHY_FILE.exists() else None

    try:
        while True:
            frame, _ = grab.latest()
            if frame is None:
                if cv2.waitKey(50) & 0xFF in (ord("q"), 27):
                    break
                continue
            g = lambda n: cv2.getTrackbarPos(n, WIN)
            C.HSV_LOWER = (g("H low"), g("S low"), g("V low"))
            C.HSV_UPPER = (g("H high"), g("S high"), g("V high"))
            C.HSV_MODE = "background" if g("mode: 0 bg / 1 obj") == 0 else "object"

            mask = vision.hsv_mask(frame)
            view = frame.copy()
            H = None
            if mapper is not None:
                mapper.set_frame_size(frame.shape[1], frame.shape[0])
                H = mapper.matrix()
            for b in vision.find_blobs(mask, H):
                color = vision.COLORS.get(b["shape"], vision.COLORS["unknown"])
                cv2.drawContours(view, [b["contour"]], -1, color, 2)
                x, y = int(b["cu"]), int(b["cv"])
                cv2.putText(view, b["shape"], (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(view, f"el {b['elongation']:.1f} hole {b['hole_frac']:.2f} rad {b['radial_spread']:.2f}",
                            (x + 10, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)
            both = np.hstack([view, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
            cv2.imshow(WIN, both)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                print(f'HSV_MODE = "{C.HSV_MODE}"')
                print(f"HSV_LOWER = {tuple(C.HSV_LOWER)}")
                print(f"HSV_UPPER = {tuple(C.HSV_UPPER)}")
    finally:
        grab.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
