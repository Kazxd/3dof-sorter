"""
collect_images.py - capture training images from the real setup.

    python collect_images.py              saves to ../datasets/own_raw/

The model trained only on internet photos will struggle with your mat, lighting
and phone. 250-400 frames from the real rig, labelled, fixes most of that.

space = save this frame      a = auto-save every 1.5 s on/off      q = quit

Vary between shots: number of parts (1-15), positions, rotations, parts touching,
nuts upside down, bolts with heads both ways, room light on/off, and a few frames
with the magnet housing or your hand in view (as negatives). Then label in Roboflow or
CVAT with exactly three classes: washer, nut, bolt, export "YOLOv8", unzip into
../datasets/own/ and run get_model.py.
"""
import argparse
import time
from pathlib import Path

import cv2

from camera import FrameGrabber

OUT = Path(__file__).resolve().parent.parent / "datasets" / "own_raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera")
    ap.add_argument("--every", type=float, default=1.5)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    grab = FrameGrabber(args.camera)
    grab.start()
    auto, last, saved = False, 0.0, len(list(OUT.glob("*.jpg")))
    try:
        while True:
            frame, _ = grab.latest()
            key = cv2.waitKey(30) & 0xFF
            if frame is None:
                if key in (ord("q"), 27):
                    break
                continue
            now = time.time()
            if key == ord(" ") or (auto and now - last >= args.every):
                name = OUT / f"img_{time.strftime('%Y%m%d_%H%M%S')}_{int(now * 1000) % 1000:03d}.jpg"
                cv2.imwrite(str(name), frame)
                saved, last = saved + 1, now
            if key == ord("a"):
                auto = not auto
            if key in (ord("q"), 27):
                break
            view = frame.copy()
            cv2.putText(view, f"saved {saved}   auto {'ON' if auto else 'off'}   space/a/q",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("collect", view)
    finally:
        grab.stop()
        cv2.destroyAllWindows()
    print(f"{saved} images in {OUT}")


if __name__ == "__main__":
    main()
