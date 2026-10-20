"""
sorter.py - main program: look, pick, sort, repeat.

    python sorter.py                    real arm, camera from config.py
    python sorter.py --sim              no arm: packets are printed, motion is simulated
    python sorter.py --no-arm           vision only: live detections + reachability, no motion
    python sorter.py --camera scene.jpg --sim --cycles 3 --headless     offline test

One cycle (the camera only looks while the arm is parked out of view):

    PARK -> SCAN (N fresh frames, keep parts seen in most of them)
         -> SELECT (inside pick area, has a bin, not failed before; isolated + nearest first)
         -> PLAN (IK for every waypoint; unreachable parts are skipped, never attempted)
         -> PICK (hover, descend, grip, lift) -> PLACE (bin, lower, release, retreat)
         -> PARK -> VERIFY (the next scan: is the part still there?) -> repeat

Keys in the window: q / Esc quit (after parking), space pause, r forget skipped spots.
"""
import argparse
import csv
import logging
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

import config as C
import kinematics as K
from arm_link import ArmError, ArmLink, SimArm
from camera import FrameGrabber
from mapping import PlaneMapper, in_workspace, mapper_from_aruco
from vision import COLORS, Detector, draw_detection

log = logging.getLogger("sorter")
WINDOW = "sorter"


@dataclass
class Target:
    label: str
    x: float                      # mm, robot frame
    y: float
    axis: Optional[float]         # long-axis direction on the table, degrees [0, 180)
    conf: float
    source: str
    seen: float                   # fraction of scan frames it appeared in
    isolated: bool = True
    note: str = ""


# =============================================================================
# multi-frame consensus
# =============================================================================
def consensus(per_frame, n_frames):
    """per_frame: one list per frame of (Detection, x_mm, y_mm, axis_mm).
    Groups detections of the same physical part across frames and keeps parts
    seen in at least MIN_FRAME_FRAC of the frames. Position = median, label =
    confidence-weighted vote, axis = circular mean (axes repeat every 180 deg)."""
    clusters = []
    for fi, items in enumerate(per_frame):
        for det, x, y, axis in items:
            best, best_d = None, C.CLUSTER_MM
            for c in clusters:
                d = math.hypot(x - np.mean(c["x"]), y - np.mean(c["y"]))
                if d < best_d:
                    best, best_d = c, d
            if best is None:
                best = {"x": [], "y": [], "frames": set(), "votes": Counter(),
                        "axes": [], "conf": [], "src": Counter()}
                clusters.append(best)
            best["x"].append(x)
            best["y"].append(y)
            best["frames"].add(fi)
            best["votes"][det.label] += det.conf
            best["conf"].append(det.conf)
            best["src"][det.source] += 1
            if axis is not None:
                best["axes"].append(axis)

    need = max(1, math.ceil(C.MIN_FRAME_FRAC * n_frames))
    targets = []
    for c in clusters:
        if len(c["frames"]) < need:
            continue
        axis = None
        if c["axes"]:
            a = np.radians(np.array(c["axes"]) * 2.0)
            axis = (math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())) / 2.0) % 180.0
        targets.append(Target(
            label=c["votes"].most_common(1)[0][0],
            x=float(np.median(c["x"])), y=float(np.median(c["y"])), axis=axis,
            conf=float(np.mean(c["conf"])), source=c["src"].most_common(1)[0][0],
            seen=len(c["frames"]) / float(n_frames)))
    for t in targets:
        others = [math.hypot(t.x - o.x, t.y - o.y) for o in targets if o is not t]
        t.isolated = not others or min(others) >= C.ISOLATION_MM
    return targets


# =============================================================================
# sorter
# =============================================================================
class Sorter:
    def __init__(self, args):
        self.args = args
        self.headless = args.headless
        self.quit = False
        self.paused = False
        self.state = "starting"
        self.targets = []
        self.last_dets = []
        self.chosen = None
        self.settled_at = 0.0
        self.skip_cells = {}            # spot -> reason it is skipped this session
        self.fail_counts = Counter()
        self.stats = Counter()
        self._aspect_warned = False

        self.mapper = self._load_mapper()
        self.cam = FrameGrabber(args.camera)
        self.cam.start()
        self.detector = Detector(use_yolo=not args.no_yolo)
        log.info("vision mode: %s", self.detector.mode)

        if args.no_arm:
            self.arm = None
        elif args.sim:
            self.arm = SimArm(speed=args.sim_speed, quiet=args.quiet_sim)
        else:
            self.arm = ArmLink(port=args.port)
        if self.arm is not None:
            self._check_bins()

        C.RUNS_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._csv_file = open(C.RUNS_DIR / f"picks_{stamp}.csv", "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["time", "label", "x_mm", "y_mm", "source", "conf", "isolated",
                            "result", "cycle_s", "note"])

    # ------------------------------------------------------------------ setup
    def _load_mapper(self):
        if C.HOMOGRAPHY_FILE.exists():
            m = PlaneMapper.load()
            log.info("homography loaded (rms %.2f mm)", m.rms_mm or float("nan"))
            return m
        if C.USE_ARUCO:
            log.info("no homography file yet - will build it from the ArUco markers")
            return None
        sys.exit(f"No camera calibration at {C.HOMOGRAPHY_FILE}. Run: python calibrate_camera.py")

    def _check_bins(self):
        """Fail at start-up, not mid-pick, if a bin cannot be reached."""
        for label, (bx, by, bz) in C.BINS.items():
            for z in (bz, bz + C.APPROACH_HEIGHT):
                try:
                    K.solve(bx, by, z)
                except K.Unreachable as e:
                    sys.exit(f"Bin '{label}' at ({bx}, {by}, {z}) is unreachable: {e}\n"
                             f"Move it in config.BINS or check the arm geometry / joint limits.")

    def _pose(self, servo_pose):
        pose = list(servo_pose)
        if C.END_EFFECTOR == "magnet":
            pose[5] = 0                 # never leave the magnet on in park / home
        return pose

    # ------------------------------------------------------------------ UI
    def pump(self):
        """Refresh the window and read keys. Called often, including during arm moves."""
        if self.headless:
            return
        frame, _ = self.cam.latest()
        if frame is None:
            frame = np.zeros((480, 640, 3), np.uint8)
        img = self.draw(frame)
        try:
            cv2.imshow(WINDOW, img)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error:
            log.warning("no GUI available (opencv-python-headless?) - continuing headless")
            self.headless = True
            return
        if key in (ord("q"), 27):
            log.info("quit requested - finishing the current move, then parking")
            self.quit = True
        elif key == ord(" "):
            self.paused = not self.paused
            log.info("paused" if self.paused else "resumed")
        elif key == ord("r"):
            self.skip_cells.clear()
            self.fail_counts.clear()
            log.info("skip list cleared")

    def draw(self, frame):
        img = frame.copy()
        h, w = img.shape[:2]
        m = self.mapper
        if m is not None:
            m.set_frame_size(w, h)
            cv2.polylines(img, [m.workspace_polygon_px()], True, (200, 200, 200), 1)
            for label, (bx, by, _) in C.BINS.items():
                u, v = m.mm_to_px(bx, by)
                if 0 <= u < w and 0 <= v < h:
                    cv2.rectangle(img, (int(u) - 12, int(v) - 12), (int(u) + 12, int(v) + 12),
                                  COLORS[label], 2)
                    cv2.putText(img, f"bin {label}", (int(u) - 12, int(v) - 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLORS[label], 1)
            if self.arm is None or self.state.startswith("scan"):
                for d in self.last_dets:
                    draw_detection(img, d)
            for t in self.targets:
                u, v = m.mm_to_px(t.x, t.y)
                color = COLORS.get(t.label, COLORS["unknown"])
                if t.note:
                    color = (0, 0, 255)
                cv2.circle(img, (int(u), int(v)), 16 if t is self.chosen else 10, color,
                           3 if t is self.chosen else 1)
                if t.axis is not None:
                    a = math.radians(t.axis)
                    p0 = m.mm_to_px(t.x - 10 * math.cos(a), t.y - 10 * math.sin(a))
                    p1 = m.mm_to_px(t.x + 10 * math.cos(a), t.y + 10 * math.sin(a))
                    cv2.line(img, tuple(map(int, p0)), tuple(map(int, p1)), color, 1)
                text = f"{t.label} ({t.x:.0f},{t.y:.0f}) {t.source}"
                if t.note:
                    text += f" [{t.note}]"
                cv2.putText(img, text, (int(u) + 14, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.42, color, 1, cv2.LINE_AA)
        bar = (f"{self.state}{'  [PAUSED]' if self.paused else ''}  |  {self.detector.mode}  |  "
               f"picked {self.stats['ok']}  failed {self.stats['failed']}  "
               f"skipped {self.stats['unreachable']}  |  cam {'ok' if self.cam.alive() else 'DOWN'}")
        cv2.rectangle(img, (0, 0), (w, 22), (0, 0, 0), -1)
        cv2.putText(img, bar, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def idle(self, seconds):
        end = time.time() + seconds
        while time.time() < end and not self.quit:
            self.pump()
            time.sleep(0.03)

    # ------------------------------------------------------------------ vision
    def scan(self):
        """Look at the table. Returns a list of Targets, or None if the camera is not usable."""
        self.state = "scan"
        if not self.cam.alive():
            self.state = "waiting for camera"
            log.warning("camera is down - arm stays parked until frames come back")
            self.idle(C.CAMERA_RECONNECT_S)
            return None
        n = C.SCAN_FRAMES if self.arm is not None else 3
        not_before = self.settled_at if self.arm is not None else time.time()
        frames = self.cam.fresh_frames(n, not_before=not_before,
                                       timeout=max(0.0, not_before - time.time()) + 6.0)
        if len(frames) < max(1, n // 2):
            log.warning("only %d/%d frames arrived during the scan", len(frames), n)
            return None

        if C.USE_ARUCO:
            fresh = mapper_from_aruco(frames[-1])
            if fresh is not None:
                self.mapper = fresh
            elif self.mapper is None:
                log.error("fewer than 4 ArUco markers visible and no saved calibration")
                return None

        h, w = frames[0].shape[:2]
        if not self.mapper.set_frame_size(w, h):
            if not self._aspect_warned:
                log.error("stream is %dx%d but calibration was %dx%d with a different aspect "
                          "ratio - set the phone back to the calibration resolution or recalibrate",
                          w, h, *self.mapper.image_size)
                self._aspect_warned = True
            return None
        roi = self.mapper.workspace_polygon_px(margin_mm=30.0)

        per_frame = []
        for f in frames:
            dets, _ = self.detector.detect(f, roi, self.mapper.matrix())
            items = []
            for d in dets:
                x, y = self.mapper.px_to_mm(d.u, d.v)
                axis = self.mapper.axis_px_to_mm(d.u, d.v, d.angle_px) if d.angle_px is not None else None
                items.append((d, x, y, axis))
            per_frame.append(items)
        self.last_dets = [it[0] for it in per_frame[-1]]
        self.targets = consensus(per_frame, len(frames))
        if self.targets:
            log.info("scan: %s", ", ".join(f"{t.label}({t.x:.0f},{t.y:.0f})" for t in self.targets))
        return self.targets

    # ------------------------------------------------------------------ planning
    @staticmethod
    def _cell(label, x, y):
        return (label, int(round(x / 10.0)), int(round(y / 10.0)))

    def plan(self, t):
        """Every waypoint of one pick-and-place as (name, 6-value servo pose, ms). Raises Unreachable."""
        g = C.GRASP[t.label]
        closing = None
        if g["roll"] == "perpendicular" and t.axis is not None:
            closing = (t.axis + 90.0) % 180.0          # jaws close across the long axis

        def at(x, y, z, closing_axis, prefer_pitch):
            try:                                       # same pitch as the grasp = straight descent
                return K.solve(x, y, z, closing_axis, pitches=[prefer_pitch])
            except K.Unreachable:
                return K.solve(x, y, z, closing_axis)

        grasp = K.solve(t.x, t.y, g["z"], closing)
        hover = at(t.x, t.y, g["z"] + C.APPROACH_HEIGHT, closing, grasp.pitch)
        lift = at(t.x, t.y, g["z"] + C.LIFT_HEIGHT, closing, grasp.pitch)
        bx, by, bz = C.BINS[t.label]
        drop = K.solve(bx, by, bz)
        above = at(bx, by, bz + C.APPROACH_HEIGHT, None, drop.pitch)

        op, cl = g["open"], g["close"]

        def pose(sol, grip):
            return list(sol.servo) + [grip]

        return [
            ("approach", pose(hover, op), C.MOVE_MS),
            ("descend", pose(grasp, op), C.DESCEND_MS),
            ("grip", pose(grasp, cl), C.GRIP_MS),
            ("lift", pose(lift, cl), C.DESCEND_MS),
            ("to bin", pose(above, cl), C.MOVE_MS),
            ("lower", pose(drop, cl), C.DESCEND_MS),
            ("release", pose(drop, op), C.GRIP_MS),
            ("retreat", pose(above, op), C.DESCEND_MS),
        ]

    def select(self, targets):
        """Best pickable target and its plan, or (None, None)."""
        candidates = []
        for t in targets:
            if not in_workspace(t.x, t.y):
                t.note = "outside pick area"
                continue
            if t.label not in C.BINS or t.label not in C.GRASP:
                t.note = "no bin for this class"
                continue
            reason = self.skip_cells.get(self._cell(t.label, t.x, t.y))
            if reason:
                t.note = reason
                continue
            candidates.append(t)
        # isolated parts first, YOLO-confirmed before fallback-only, then nearest to the base
        candidates.sort(key=lambda t: (not t.isolated, t.source == "hsv", math.hypot(t.x, t.y)))
        for t in candidates:
            try:
                return t, self.plan(t)
            except K.Unreachable as e:
                t.note = "unreachable"
                self.skip_cells[self._cell(t.label, t.x, t.y)] = "unreachable"
                self.stats["unreachable"] += 1
                log.warning("skipping %s at (%.0f, %.0f): %s", t.label, t.x, t.y, e)
                self._record(t, "unreachable", 0.0, str(e))
        return None, None

    # ------------------------------------------------------------------ motion
    def park(self):
        self.state = "park"
        self.arm.move(self._pose(C.PARK_SERVO), C.PARK_MS, on_wait=self.pump)
        self.settled_at = time.time() + C.SCAN_SETTLE_S

    def execute(self, target, steps):
        for name, pose, ms in steps:
            self.state = f"{name} {target.label}"
            self.arm.move(pose, ms, on_wait=self.pump)

    def recover(self):
        """After a serial error: try to stop, then reconnect once."""
        log.error("arm link problem - trying to recover")
        try:
            self.arm.stop()
            return True
        except ArmError:
            pass
        try:
            self.arm.close()
            self.arm.connect()
            self.park()
            return True
        except ArmError as e:
            log.error("could not recover the arm link: %s", e)
            return False

    # ------------------------------------------------------------------ bookkeeping
    def verify(self, t, after):
        if after is None:
            return "unverified"
        for o in after:
            if o.label == t.label and math.hypot(o.x - t.x, o.y - t.y) < C.VERIFY_RADIUS_MM:
                return "failed"
        return "ok"

    def _record(self, t, result, cycle_s, note=""):
        self._csv.writerow([time.strftime("%H:%M:%S"), t.label, f"{t.x:.1f}", f"{t.y:.1f}",
                            t.source, f"{t.conf:.2f}", int(t.isolated), result, f"{cycle_s:.1f}", note])
        self._csv_file.flush()

    def annotate_reachability(self, targets):
        """Vision-only mode: show what the arm *would* do with each part."""
        for t in targets:
            if not in_workspace(t.x, t.y):
                t.note = "outside pick area"
                continue
            try:
                steps = self.plan(t)
                t.note = ""
                log.debug("%s ok: grasp pose %s", t.label, steps[1][1])
            except K.Unreachable as e:
                t.note = "unreachable"
                log.debug("%s unreachable: %s", t.label, e)
            except KeyError:
                t.note = "no bin for this class"

    # ------------------------------------------------------------------ main loop
    def run(self):
        if self.arm is not None:
            self.state = "connecting"
            self.arm.connect()
            self.park()

        cycles = 0
        targets = None
        while not self.quit:
            if self.paused:
                self.idle(0.1)
                continue
            if targets is None:
                targets = self.scan()
            if targets is None:                              # camera trouble
                self.idle(0.5)
                continue

            if self.arm is None:                             # vision-only mode
                self.annotate_reachability(targets)
                self.idle(0.05)
                targets = None
                continue

            t, steps = self.select(targets)
            if t is None:
                self.chosen = None
                self.state = "nothing to pick"
                self.idle(C.IDLE_WAIT_S)
                targets = None
                continue

            self.chosen = t
            log.info("picking %s at (%.0f, %.0f) via %s", t.label, t.x, t.y, t.source)
            t0 = time.time()
            try:
                self.execute(t, steps)
                self.park()
            except ArmError as e:
                log.error("arm error: %s", e)
                self._record(t, "arm-error", time.time() - t0, str(e))
                if not self.recover():
                    break
                targets = None
                continue

            targets = self.scan()                            # doubles as verification
            result = self.verify(t, targets)
            cycle_s = time.time() - t0
            self.stats[result] += 1
            if result == "failed":
                cell = self._cell(t.label, t.x, t.y)
                self.fail_counts[cell] += 1
                if self.fail_counts[cell] >= C.MAX_ATTEMPTS:
                    self.skip_cells[cell] = f"failed {C.MAX_ATTEMPTS}x"
                    log.warning("giving up on the %s at (%.0f, %.0f)", t.label, t.x, t.y)
            log.info("pick %s: %s in %.1f s", t.label, result, cycle_s)
            self._record(t, result, cycle_s)
            self.chosen = None

            cycles += 1
            if self.args.cycles and cycles >= self.args.cycles:
                break

    def shutdown(self):
        if self.arm is not None:
            try:
                self.park()
            except Exception as e:
                log.warning("could not park on exit: %s", e)
            self.arm.close()
        self.cam.stop()
        self._csv_file.close()
        if not self.headless:
            cv2.destroyAllWindows()
        log.info("done - picked %d, failed %d, unverified %d, unreachable %d",
                 self.stats["ok"], self.stats["failed"], self.stats["unverified"],
                 self.stats["unreachable"])


# =============================================================================
def setup_logging(verbose=False):
    C.RUNS_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)-7s %(message)s", datefmt="%H:%M:%S",
        handlers=[logging.StreamHandler(), logging.FileHandler(C.RUNS_DIR / "sorter.log", encoding="utf-8")])


def main(argv=None):
    p = argparse.ArgumentParser(description="Vision-guided 5-DOF fastener sorter")
    p.add_argument("--camera", help="override CAMERA_URL (URL, webcam index, image or video file)")
    p.add_argument("--port", help="override SERIAL_PORT")
    p.add_argument("--sim", action="store_true", help="simulate the arm (no serial)")
    p.add_argument("--sim-speed", type=float, default=4.0, help="simulation speed-up")
    p.add_argument("--quiet-sim", action="store_true", help="don't print simulated packets")
    p.add_argument("--no-arm", action="store_true", help="vision + IK preview only")
    p.add_argument("--no-yolo", action="store_true", help="force the HSV fallback")
    p.add_argument("--headless", action="store_true", help="no window")
    p.add_argument("--cycles", type=int, default=0, help="stop after N pick attempts")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)
    setup_logging(args.verbose)

    sorter = Sorter(args)
    try:
        sorter.run()
    except KeyboardInterrupt:
        log.info("Ctrl+C - parking and exiting")
    except ArmError as e:
        log.error("stopping: %s", e)
    finally:
        sorter.shutdown()
    return sorter


if __name__ == "__main__":
    main()
