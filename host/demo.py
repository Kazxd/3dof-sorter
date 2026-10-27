"""
demo.py - scripted pick-and-place demo with the live camera view
================================================================
The parts sit at fixed, measured spots (PARTS below) instead of being found by the
vision pipeline. Every move is planned exactly like sorter.py does it (same IK, same
waypoints, same bins), and the camera window shows the table live, with each pick spot,
its drop spot and an arrow between them once the camera is calibrated.

    python demo.py --port auto --calibrate      one-off: the arm shows the camera where it
                                                reaches; click the magnet each time it stops
    python demo.py --port auto                  camera from CAMERA_URL in config.py
    python demo.py --port auto --camera 0       laptop / USB webcam instead
    python demo.py --port auto --no-camera      arm only, no window
    python demo.py --sim                        no hardware at all

Window keys: SPACE starts the run, q stops it (the arm goes home with the magnet off).
"""
import argparse
import logging
import sys
import time

import cv2
import numpy as np

import config as C
import kinematics as K
import sorter as S
from arm_link import ArmLink, SimArm
from camera import FrameGrabber
from mapping import PlaneMapper

log = logging.getLogger("demo")

# (label, X mm, Y mm) from the table point under the base axis; +X ahead, +Y to the left.
# Picked in this order; each goes to its bin in config.BINS.
PARTS = [
    ("nut",  185.0,   0.0),    # 18.5 cm straight ahead
    ("bolt", 165.0, -77.0),    # 16.5 cm ahead, 7.7 cm to the right (18.2 cm out)
]
# Where the arm parks the magnet (just above the table) during --calibrate, as
# (distance from the base axis mm, angle deg; + = left). Spread over everything it reaches.
CALIB_SPOTS = [(178, -55), (188, -25), (178, 0), (188, 25), (178, 55), (186, 80)]
CALIB_Z = 4.0
WINDOW = "Sorter demo"
COLOURS = {"waiting": (0, 220, 255), "active": (0, 255, 0), "done": (160, 160, 160)}


class Demo:
    def __init__(self, args):
        self.arm = SimArm(speed=args.sim_speed, quiet=True) if args.sim else ArmLink(port=args.port)
        self.cam = None
        if not args.no_camera:
            self.cam = FrameGrabber(args.camera)
            self.cam.start()
        self.mapper = PlaneMapper.load() if C.HOMOGRAPHY_FILE.exists() else None
        self._size = None
        self.state = {label: "waiting" for label, _x, _y in PARTS}
        self.status = "ready - press SPACE to start"
        self.started = self.cam is None
        self.quit = False
        self.clicks = []
        if self.cam is not None:
            cv2.namedWindow(WINDOW)
            cv2.setMouseCallback(WINDOW, self._on_mouse)

    def _on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))

    # ------------------------------------------------------------------ window
    def pump(self):
        """Refresh the window and read keys. Called during every arm move."""
        if self.cam is None:
            return
        frame, _ts = self.cam.latest()
        if frame is None:
            frame = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(frame, "waiting for camera...", (20, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            self.draw(frame)
        cv2.imshow(WINDOW, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            self.started = True
        elif key in (ord("q"), 27):
            self.quit = True

    def draw(self, frame):
        h, w = frame.shape[:2]
        if self.mapper is not None:
            if self._size != (w, h):
                self.mapper.set_frame_size(w, h)
                self._size = (w, h)
            for label, x, y in PARTS:
                u, v = (int(round(c)) for c in self.mapper.mm_to_px(x, y))
                colour = COLOURS[self.state[label]]
                bx, by, _bz = C.BINS[label]
                bu, bv = (int(round(c)) for c in self.mapper.mm_to_px(bx, by))
                cv2.arrowedLine(frame, (u, v), (bu, bv), colour, 2, tipLength=0.06)
                cv2.circle(frame, (u, v), 18, colour, 2)
                cv2.putText(frame, label, (u + 22, v + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            for label in {label for label, _x, _y in PARTS}:
                bx, by, _bz = C.BINS[label]
                u, v = (int(round(c)) for c in self.mapper.mm_to_px(bx, by))
                cv2.rectangle(frame, (u - 16, v - 16), (u + 16, v + 16), (255, 160, 0), 2)
                cv2.putText(frame, f"{label} bin", (u + 20, v + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 160, 0), 1)
        cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(frame, self.status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    def wait(self, seconds=None):
        """Keep the window live until SPACE (seconds=None) or for a fixed time."""
        end = None if seconds is None else time.time() + seconds
        while not self.quit and (not self.started if end is None else time.time() < end):
            self.pump()
            time.sleep(0.03)

    # ------------------------------------------------------------------ motion
    def home(self):
        self.arm.move(list(C.HOME_SERVO[:5]) + [0], C.PARK_MS, on_wait=self.pump)

    def run(self):
        planner = S.Sorter.__new__(S.Sorter)          # plan() needs no camera or arm
        plans = []
        for label, x, y in PARTS:
            t = S.Target(label=label, x=x, y=y, axis=None, conf=1.0, source="demo", seen=time.time())
            try:
                plans.append((label, planner.plan(t)))
            except K.Unreachable as e:
                sys.exit(f"{label} at ({x:.0f}, {y:.0f}) mm is out of reach: {e}")

        self.arm.connect()
        self.home()
        self.wait()
        for label, steps in plans:
            self.state[label] = "active"
            for name, pose, ms in steps:
                if self.quit:
                    return
                self.status = f"{label}: {name}"
                log.info(self.status)
                self.arm.move(pose, ms, on_wait=self.pump)
            self.state[label] = "done"
            self.status = f"{label}: sorted - going home"
            self.home()
        self.status = "done - all parts sorted (q to close)"
        log.info(self.status)
        if self.cam is not None:
            self.started = False
            self.wait()

    def calibrate(self):
        """The arm parks the magnet at known spots; a click on it in the picture pairs
        pixel and millimetres. Saves the homography that sorter.py and this demo use."""
        import math
        if self.cam is None:
            sys.exit("--calibrate needs the camera")
        self.arm.connect()
        px, mm = [], []
        for i, (r, ang) in enumerate(CALIB_SPOTS, 1):
            x, y = r * math.cos(math.radians(ang)), r * math.sin(math.radians(ang))
            self.status = f"point {i}/{len(CALIB_SPOTS)}: moving..."
            sol = K.solve(x, y, CALIB_Z)
            self.arm.move(list(sol.servo) + [0], C.MOVE_MS, on_wait=self.pump)
            self.status = f"point {i}/{len(CALIB_SPOTS)}: click the centre of the magnet"
            self.clicks.clear()
            while not self.clicks and not self.quit:
                self.pump()
                time.sleep(0.03)
            if self.quit:
                return
            px.append(self.clicks[-1])
            mm.append((x, y))
            log.info("point %d: (%.0f, %.0f) mm -> pixel %s", i, x, y, self.clicks[-1])
        frame, _ = self.cam.latest()
        h, w = frame.shape[:2]
        self.mapper = PlaneMapper.from_points(px, mm, (w, h))
        self.mapper.save(extra={"points_px": [list(map(float, c)) for c in px],
                                "points_mm": [list(c) for c in mm], "method": "arm-guided"})
        self._size = None
        log.info("camera calibrated: %d points, rms %.1f mm -> %s",
                 len(px), self.mapper.rms_mm or 0.0, C.HOMOGRAPHY_FILE)
        self.home()
        self.status = f"calibrated (rms {self.mapper.rms_mm or 0:.1f} mm) - q to close"
        self.started = False
        self.wait()

    def shutdown(self):
        try:
            self.home()
        except Exception as e:                       # noqa: BLE001
            log.warning("could not go home: %s", e)
        self.arm.close()
        if self.cam is not None:
            self.cam.stop()
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def main(argv=None):
    p = argparse.ArgumentParser(description="Scripted pick-and-place demo with the live camera view")
    p.add_argument("--port", help="override SERIAL_PORT (e.g. auto, COM14)")
    p.add_argument("--camera", help="override CAMERA_URL (URL or webcam index)")
    p.add_argument("--no-camera", action="store_true", help="no camera window")
    p.add_argument("--sim", action="store_true", help="simulate the arm (no serial)")
    p.add_argument("--sim-speed", type=float, default=4.0)
    p.add_argument("--calibrate", action="store_true",
                   help="calibrate the camera: click the magnet as the arm parks it at known spots")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")
    demo = Demo(args)
    try:
        demo.calibrate() if args.calibrate else demo.run()
    except KeyboardInterrupt:
        log.info("stopped")
    finally:
        demo.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
