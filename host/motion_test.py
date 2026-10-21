"""
motion_test.py - check that the arm moves correctly, one stage at a time.

    python motion_test.py --sim                 dry run, no hardware
    python motion_test.py                       real arm; asks before every stage
    python motion_test.py --stages joints       only some stages (comma separated)
    python motion_test.py --amp 10              smaller joint swings (default 20 deg)
    python motion_test.py --yes                 no questions (still starts slowly from home)

Stages, in order:
    link      ping the board, read its pose
    home      go to HOME_SERVO slowly
    joints    swing each servo + and - around home, one at a time
    magnet    switch the electromagnet on for 1 s, then off
    ik        move the magnet face to points over the pick area at a safe height
    speed     ask for a fast base move and check the firmware slows it down
    repeat    shuttle between two points and check every move finishes on time
    park      go to PARK_SERVO

Before you start: clear everything off the mat, keep a hand on the servo power
switch, and stand the arm near its HOME pose (servos jump there at power-up).
Ctrl+C at any time stops the arm and brings it home slowly.

A report is saved to runs/motion_test_<time>.txt.
"""
import argparse
import logging
import sys
import time

import config as C
import kinematics as K
from arm_link import ArmError, ArmLink, SimArm

STAGES = ["link", "home", "joints", "magnet", "ik", "speed", "repeat", "park"]
JOINT_LABELS = ["base", "shoulder", "elbow", "wrist pitch", "wrist roll"]
POSITIVE_MEANS = [                       # what a positive kinematic angle should look like
    "turns LEFT (counter-clockwise seen from above)",
    "LIFTS the upper arm",
    "bends the forearm UP",
    "tilts the magnet head UP",
    "turns the head counter-clockwise (seen from below)",
]
SLOW_MS = 1500
SAFE_Z = 60.0                            # mm above the table for the IK points


class Tester:
    def __init__(self, arm, ask, amp):
        self.arm = arm
        self.ask_enabled = ask
        self.amp = amp
        self.pose = self._home()
        self.results = []                # (stage, status, detail)
        self.notes = []

    # ------------------------------------------------------------- helpers
    @staticmethod
    def _home():
        home = list(C.HOME_SERVO)
        home[5] = 0
        return home

    def ask(self, question, default=True):
        if not self.ask_enabled:
            return default
        hint = "Y/n" if default else "y/N"
        while True:
            ans = input(f"  {question} [{hint}] ").strip().lower()
            if not ans:
                return default
            if ans in ("y", "yes"):
                return True
            if ans in ("n", "no"):
                return False

    def move(self, pose, ms=SLOW_MS):
        t0 = time.time()
        real = self.arm.move(pose, ms)
        self.pose = list(pose)
        return real, time.time() - t0

    def joint_window(self, i):
        """Servo range for joint i: home +/- amp, clipped to the limits in config.JOINTS."""
        j = C.JOINTS[K.JOINT_NAMES[i]]
        home = C.HOME_SERVO[i]
        return home, max(j["min"], home - self.amp), min(j["max"], home + self.amp)

    def record(self, stage, status, detail=""):
        self.results.append((stage, status, detail))
        print(f"  -> {status.upper():4s}  {detail}")

    # ------------------------------------------------------------- stages
    def stage_link(self):
        self.arm.ping()
        pose, moving = self.arm.query()
        self.record("link", "pass", "board answered; pose " + ", ".join(f"{v:.0f}" for v in pose)
                    + (" (moving)" if moving else ""))

    def stage_home(self):
        _, took = self.move(self._home(), 3000)
        self.record("home", "pass", f"reached HOME in {took:.1f} s")

    def stage_joints(self):
        for i, name in enumerate(JOINT_LABELS):
            home, lo, hi = self.joint_window(i)
            if hi - lo < 2:
                self.record(f"joint {i}", "skip", f"{name}: no room inside its limits")
                continue
            print(f"\n  [{i}] {name}: {home:.0f} -> {hi:.0f} -> {lo:.0f} -> {home:.0f} (servo deg)")
            if not self.ask(f"swing the {name}? (say no if this servo isn't fitted)"):
                self.record(f"joint {i}", "skip", name)
                continue
            try:
                for target in (hi, lo, home):
                    pose = self._home()
                    pose[i] = target
                    self.move(pose, SLOW_MS)
            except ArmError as e:
                self.record(f"joint {i}", "fail", f"{name}: {e}")
                continue

            # raising the servo angle must move the joint the positive kinematic way when direction = +1
            sign = C.JOINTS[K.JOINT_NAMES[i]]["direction"]
            expect = POSITIVE_MEANS[i] if sign > 0 else "does the OPPOSITE of: " + POSITIVE_MEANS[i]
            if self.ask(f"on the first swing (servo angle going UP), the {name} {expect}?"):
                self.record(f"joint {i}", "pass", f"{name} moves and its direction matches config")
            else:
                self.record(f"joint {i}", "fail", f"{name}: set JOINTS['{K.JOINT_NAMES[i]}']"
                                                  f"['direction'] = {-sign:+d} in config.py")
            if self.ask_enabled and not self.ask("smooth start and stop, no buzzing or jerking?"):
                self.notes.append(f"{name}: rough motion - check the horn screw, wiring and supply current")

    def stage_magnet(self):
        if not self.ask("ready to switch the magnet on? (a steel nut or screwdriver tip nearby helps)"):
            self.record("magnet", "skip")
            return
        on = list(self.pose)
        on[5] = 1
        self.move(on, 300)
        time.sleep(1.0)
        off = list(self.pose)
        off[5] = 0
        self.move(off, 300)
        ok = self.ask("did the magnet grab and then let go?")
        self.record("magnet", "pass" if ok else "fail",
                    "magnet switches" if ok else "check MOSFET wiring, diode, 12 V supply and MAGNET_PIN")

    def stage_ik(self):
        (x0, x1), (y0, y1) = C.WORKSPACE_X, C.WORKSPACE_Y
        xc, yc = (x0 + x1) / 2, (y0 + y1) / 2
        points = [("centre", xc, yc), ("near-left", x0, y1), ("far-left", x1, y1),
                  ("far-right", x1, y0), ("near-right", x0, y0), ("centre", xc, yc)]
        print(f"\n  The magnet face will visit {len(points)} points at Z = {SAFE_Z:.0f} mm above the table.")
        print("  Use a ruler: the face should be about that high, over the printed X/Y position.")
        if not self.ask("run the IK points?"):
            self.record("ik", "skip")
            return
        reached = 0
        for name, x, y in points:
            try:
                sol = K.solve(x, y, SAFE_Z)
            except K.Unreachable as e:
                self.record("ik", "note", f"{name} ({x:.0f}, {y:.0f}) unreachable: {e}")
                continue
            _, took = self.move(list(sol.servo) + [0], C.MOVE_MS)
            fx, fy, fz, pitch = K.forward_kinematics(sol.kin)
            print(f"    {name:10s} X={fx:6.1f} Y={fy:6.1f} Z={fz:5.1f}  pitch {pitch:4.0f}  "
                  f"servo {[round(v) for v in sol.servo]}  {took:.1f} s")
            reached += 1
            if self.ask_enabled:
                input("    measure, then press Enter for the next point ")
        good = reached > 0 and self.ask("were the positions within about 5 mm of the targets?")
        self.record("ik", "pass" if good else "fail", f"{reached}/{len(points)} points reached" +
                    ("" if good else "; re-measure L1-L3 / TOOL_* and the servo zeros"))
        self.move(self._home(), SLOW_MS)

    def stage_speed(self):
        home, lo, hi = self.joint_window(0)
        if hi - lo < 10:
            self.record("speed", "skip", "base has no room")
            return
        if not self.ask("ask for a FAST base swing (the firmware should slow it down)?"):
            self.record("speed", "skip")
            return
        pose = self._home()
        pose[0] = hi
        asked = 200
        real, took = self.move(pose, asked)
        self.move(self._home(), SLOW_MS)
        if real is None:
            self.record("speed", "note", f"board did not report a duration; the move took {took:.2f} s")
        elif real > asked:
            self.record("speed", "pass", f"asked {asked} ms, firmware stretched it to {real} ms: speed cap works")
        else:
            self.record("speed", "fail", f"asked {asked} ms and the board kept {real} ms: check MAX_SPEED_DPS")

    def stage_repeat(self, cycles=5):
        (x0, x1), (y0, y1) = C.WORKSPACE_X, C.WORKSPACE_Y
        a = ((x0 + x1) / 2, y0 + 0.25 * (y1 - y0))
        b = ((x0 + x1) / 2, y0 + 0.75 * (y1 - y0))
        try:
            sa, sb = K.solve(a[0], a[1], SAFE_Z), K.solve(b[0], b[1], SAFE_Z)
        except K.Unreachable as e:
            self.record("repeat", "skip", f"test points unreachable: {e}")
            return
        if not self.ask(f"shuttle {cycles}x between ({a[0]:.0f}, {a[1]:.0f}) and ({b[0]:.0f}, {b[1]:.0f})?"):
            self.record("repeat", "skip")
            return
        times = []
        for _ in range(cycles):
            for sol in (sa, sb):
                times.append(self.move(list(sol.servo) + [0], C.MOVE_MS)[1])
        self.record("repeat", "pass", f"{len(times)} moves, {min(times):.2f}-{max(times):.2f} s each")
        if self.ask_enabled and not self.ask("did it come back to the same spot every time (within ~3 mm)?"):
            self.notes.append("repeatability poor: tighten horn screws, check gear slop or a sagging shoulder")
        self.move(self._home(), SLOW_MS)

    def stage_park(self):
        self.move(list(C.PARK_SERVO[:5]) + [0], C.PARK_MS)
        ok = self.ask("is the whole arm now out of the camera's view?")
        self.record("park", "pass" if ok else "fail", "" if ok else "adjust PARK_SERVO in config.py")

    # ------------------------------------------------------------- run
    def safe_stop(self):
        try:
            self.arm.stop()
            self.arm.move(self._home(), 3000)
        except Exception as e:
            print(f"  could not bring the arm home: {e} - switch the servo power off")

    def run(self, stages):
        for name in stages:
            print(f"\n=== {name} ===")
            try:
                getattr(self, f"stage_{name}")()
            except K.Unreachable as e:
                self.record(name, "fail", str(e))
            except ArmError as e:
                self.record(name, "fail", f"arm error: {e}")
                if name == "link":
                    return
                self.safe_stop()

    def report(self):
        lines = ["=== summary ==="]
        lines += [f"{status.upper():5s} {stage:9s} {detail}" for stage, status, detail in self.results]
        lines += [f"NOTE  {n}" for n in self.notes]
        if not self.ask_enabled:
            lines.append("NOTE  questions were skipped (--sim / --yes): direction, magnet, accuracy and "
                         "park checks were ASSUMED ok, not observed")
        fails = sum(1 for r in self.results if r[1] == "fail")
        lines.append(f"{fails} failure(s)." if fails else "All stages that ran passed.")
        text = "\n".join(lines)
        print("\n" + text)
        C.RUNS_DIR.mkdir(parents=True, exist_ok=True)
        path = C.RUNS_DIR / f"motion_test_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path.write_text(text + "\n", encoding="utf-8")
        print(f"report saved to {path}")
        return fails


def main(argv=None):
    p = argparse.ArgumentParser(description="Staged movement test for the sorter arm")
    p.add_argument("--sim", action="store_true", help="simulated arm, no hardware")
    p.add_argument("--port", help="override SERIAL_PORT")
    p.add_argument("--stages", default=",".join(STAGES), help="comma-separated subset of: " + ", ".join(STAGES))
    p.add_argument("--amp", type=float, default=20.0, help="joint swing in servo degrees (default 20)")
    p.add_argument("--yes", action="store_true", help="don't ask questions")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")

    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    bad = [s for s in stages if s not in STAGES]
    if bad:
        sys.exit(f"unknown stage(s): {', '.join(bad)}")

    interactive = not args.yes and not args.sim
    arm = SimArm(speed=20.0, quiet=True) if args.sim else ArmLink(port=args.port)
    tester = Tester(arm, ask=interactive, amp=args.amp)
    if interactive:
        print("Clear the mat, stand the arm near HOME, keep a hand on the servo power switch.")
        input("Press Enter to start (Ctrl+C aborts) ")
    try:
        arm.connect()
        tester.run(stages)
    except KeyboardInterrupt:
        print("\nCtrl+C - stopping and going home")
        tester.safe_stop()
    except ArmError as e:
        print(f"cannot talk to the arm: {e}")
        return 2
    finally:
        try:
            arm.close()
        except Exception:
            pass
    return 1 if tester.report() else 0


if __name__ == "__main__":
    sys.exit(main())
