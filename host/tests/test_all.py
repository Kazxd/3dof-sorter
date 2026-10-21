"""
Tests that run without hardware, a camera or YOLO.

    python -m pytest tests -q          (or)          python tests/test_all.py
"""
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

HOST = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HOST))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import config as C            # noqa: E402
import kinematics as K        # noqa: E402
import arm_link               # noqa: E402
import mapping                # noqa: E402
import vision                 # noqa: E402
import synthetic              # noqa: E402


# ----------------------------------------------------------------- kinematics
def test_ik_fk_round_trip():
    n = 0
    for x in range(110, 240, 10):
        for y in range(-110, 120, 20):
            for z in (0.5, 3.0, 40.0):
                try:
                    sol = K.solve(x, y, z)
                except K.Unreachable:
                    continue
                fx, fy, fz, pitch = K.forward_kinematics(sol.kin)
                assert abs(fx - x) < 1e-6 and abs(fy - y) < 1e-6 and abs(fz - z) < 1e-6
                assert abs(pitch - sol.pitch) < 1e-6
                for i, s in enumerate(sol.servo):
                    assert K.limit_violation(i, s) is None
                n += 1
    assert n > 100, f"only {n} grid points reachable - defaults inconsistent"


def test_inline_mount_round_trip():
    saved = (C.TOOL_FORWARD, C.TOOL_DOWN, C.TOOL_AXIS_DEG, dict(C.JOINTS["wrist_pitch"]))
    C.TOOL_FORWARD, C.TOOL_DOWN, C.TOOL_AXIS_DEG = 110.0, 0.0, 0.0
    C.JOINTS["wrist_pitch"].update(min=-90.0, max=270.0)          # don't let limits hide the maths
    try:
        n = 0
        for x, y, z in ((150, 0, 3), (180, 60, 3), (200, -40, 20), (140, 90, 40)):
            sol = K.solve(x, y, z)
            fx, fy, fz, pitch = K.forward_kinematics(sol.kin)
            assert max(abs(fx - x), abs(fy - y), abs(fz - z)) < 1e-6 and abs(pitch - sol.pitch) < 1e-6
            n += 1
        assert n == 4
    finally:
        C.TOOL_FORWARD, C.TOOL_DOWN, C.TOOL_AXIS_DEG = saved[:3]
        C.JOINTS["wrist_pitch"] = saved[3]


def test_prefers_vertical_tool():
    assert K.solve(160, 0, 3).pitch == -90


def test_roll_aligns_jaws():
    for axis in (0, 30, 75, 120, 170):
        sol = K.solve(170, 40, 3, closing_axis_deg=axis)
        t1, t5 = sol.kin[0], sol.kin[4]
        closing = (t1 + 90 + t5) % 180
        assert abs((closing - axis + 90) % 180 - 90) < 1e-6


def test_unreachable_reasons():
    for args, word in (((600, 0, 0), "too far"), ((-150, 0, 3), "base"), ((160, 0, -20), "floor")):
        try:
            K.solve(*args)
            raise AssertionError(f"{args} should be unreachable")
        except K.Unreachable as e:
            assert word in str(e), str(e)


# ----------------------------------------------------------------- protocol
def test_packet_round_trip():
    payload = "M,12,90.0,45.5,120.0,30.0,90.0,60.0,800"
    frame = arm_link.encode(payload).decode().strip()
    assert frame.startswith("<") and frame.endswith(">")
    assert arm_link.decode(frame[1:-1]) == payload.split(",")
    broken = frame[1:-1].replace("45.5", "46.5")
    assert arm_link.decode(broken) is None
    assert arm_link.decode("ACK,12,900") == ["ACK", "12", "900"]


def test_sim_arm_records_moves():
    arm = arm_link.SimArm(speed=1000, quiet=True)
    arm.move([90, 90, 90, 90, 90, 60], 100)
    assert arm.sent[-1].startswith("M,1,90.0")


class FakeBoard:
    """Stands in for serial.Serial: answers like the firmware, and drops the first
    ACK it owes so the host's retry + duplicate-seq handling gets exercised."""

    def __init__(self, *a, **k):
        import threading
        self.out = b"<READY,ARM5,1>\r\n"
        self.lock = threading.Lock()
        self.seen = set()
        self.dropped = False
        self.moves = 0

    def write(self, data):
        frame = data.decode().strip()[1:-1]
        f = arm_link.decode(frame)
        assert f is not None, "host sent a bad checksum"
        cmd, seq = f[0], f[1]
        reply = ""
        if cmd == "P":
            reply = f"<PONG,{seq}>"
        elif cmd == "M":
            if seq in self.seen:                     # duplicate: re-ACK, don't move again
                reply = f"<ACK,{seq}><DONE,{seq}>"
            else:
                self.seen.add(seq)
                self.moves += 1
                if not self.dropped:
                    self.dropped = True              # pretend this ACK got lost
                    return len(data)
                reply = f"<ACK,{seq},{f[-1]}><DONE,{seq}>"
        elif cmd == "Q":
            reply = f"<POS,{seq},90.0,90.0,90.0,90.0,90.0,60.0,0>"
        with self.lock:
            self.out += reply.encode()
        return len(data)

    def read(self, n):
        import time
        with self.lock:
            chunk, self.out = self.out[:n], self.out[n:]
        if not chunk:
            time.sleep(0.005)
        return chunk

    def close(self):
        pass


def test_arm_link_against_fake_board():
    if arm_link.serial is None:
        return
    real = arm_link.serial.Serial
    board = FakeBoard()
    arm_link.serial.Serial = lambda *a, **k: board
    try:
        arm = arm_link.ArmLink(port="FAKE")
        arm.connect()
        arm.move([90, 80, 70, 60, 90, 50], 200)       # first ACK is dropped -> retried
        arm.move([95, 80, 70, 60, 90, 50], 200)
        pose, moving = arm.query()
        arm.close()
    finally:
        arm_link.serial.Serial = real
    assert board.moves == 2, "a retried packet must not be executed twice"
    assert pose[0] == 90.0 and moving is False


# ----------------------------------------------------------------- mapping
def test_homography_round_trip_and_resolution_change():
    mm = synthetic.MM_PTS.tolist()
    px = synthetic.PX_PTS.tolist()
    m = mapping.PlaneMapper.from_points(px, mm, synthetic.SIZE)
    assert m.rms_mm < 1e-6
    for x, y in ((150, 0), (200, -80), (120, 90)):
        u, v = m.mm_to_px(x, y)
        bx, by = m.px_to_mm(u, v)
        assert abs(bx - x) < 1e-6 and abs(by - y) < 1e-6
    assert m.set_frame_size(synthetic.SIZE[0] // 2, synthetic.SIZE[1] // 2)
    u, v = m.mm_to_px(150, 0)
    x, y = m.px_to_mm(u, v)
    assert abs(x - 150) < 1e-6 and abs(y) < 1e-6
    assert not m.set_frame_size(1280, 720)            # 16:9 vs 4:3 must be flagged


# ----------------------------------------------------------------- vision
def _scene_detections():
    img = synthetic.make_scene()
    m = mapping.PlaneMapper(synthetic.H_PX2MM, synthetic.SIZE)
    det = vision.Detector(use_yolo=False)
    dets, _ = det.detect(img, m.workspace_polygon_px(margin_mm=30), m.matrix())
    return m, dets


def test_hsv_fallback_finds_and_classifies():
    m, dets = _scene_detections()
    assert len(dets) == len(synthetic.PARTS), [d.label for d in dets]
    for label, x, y, ang in synthetic.PARTS:
        near = [d for d in dets if math.hypot(*np.subtract(m.px_to_mm(d.u, d.v), (x, y))) < 8]
        assert len(near) == 1, f"{label} at {(x, y)} not found once"
        d = near[0]
        assert d.label == label, f"{label} classified as {d.label}"
        dx, dy = np.subtract(m.px_to_mm(d.u, d.v), (x, y))
        if label == "bolt":
            a = math.radians(ang)
            assert dx * math.cos(a) + dy * math.sin(a) > 0, "bolt grasp should shift toward the head"
            axis = m.axis_px_to_mm(d.u, d.v, d.angle_px)
            assert abs((axis - ang + 90) % 180 - 90) < 3, axis
        else:
            assert math.hypot(dx, dy) < 1.0, (label, dx, dy)


def test_class_name_mapping():
    assert vision.canonical_label("silver washer") == "washer"
    assert vision.canonical_label("Hex Nut") == "nut"
    assert vision.canonical_label("phillips-screw") == "bolt"
    assert vision.canonical_label("bolt-nut-washer") is None
    assert vision.canonical_label("gear") is None


# ----------------------------------------------------------------- consensus
def test_consensus_merges_and_filters():
    import sorter
    d = vision.Detection("nut", 0, 0, 0.9, "yolo", (0, 0, 1, 1))
    flicker = vision.Detection("bolt", 0, 0, 0.4, "yolo", (0, 0, 1, 1))
    frames = [[(d, 150 + i * 0.3, 10, None)] for i in range(5)]
    frames[0].append((flicker, 200, 50, None))              # seen once -> dropped
    t = sorter.consensus(frames, 5)
    assert len(t) == 1 and t[0].label == "nut" and abs(t[0].x - 150.6) < 1e-6


# ----------------------------------------------------------------- motion test tool
def test_motion_test_runs_in_sim():
    import motion_test
    saved = C.RUNS_DIR
    C.RUNS_DIR = Path(tempfile.mkdtemp()) / "runs"
    try:
        assert motion_test.main(["--sim", "--yes"]) == 0
        assert list(C.RUNS_DIR.glob("motion_test_*.txt")), "report not written"
    finally:
        C.RUNS_DIR = saved


# ----------------------------------------------------------------- end to end
def test_sorter_end_to_end_sim():
    import sorter
    tmp = Path(tempfile.mkdtemp())
    scene, calib = synthetic.write_files(tmp)
    saved = (C.HOMOGRAPHY_FILE, C.RUNS_DIR, C.SCAN_SETTLE_S)
    C.HOMOGRAPHY_FILE, C.RUNS_DIR, C.SCAN_SETTLE_S = calib, tmp / "runs", 0.05
    try:
        s = sorter.main(["--camera", str(scene), "--sim", "--sim-speed", "200", "--quiet-sim",
                         "--headless", "--no-yolo", "--cycles", "3"])
    finally:
        C.HOMOGRAPHY_FILE, C.RUNS_DIR, C.SCAN_SETTLE_S = saved
    moves = [p for p in s.arm.sent if p.startswith("M,")]
    # park + 3 x (8 waypoints + park) + final park
    assert len(moves) == 1 + 3 * 9 + 1, len(moves)
    for p in moves:
        vals = [float(v) for v in p.split(",")[2:8]]
        for i in range(5):
            assert K.limit_violation(i, vals[i]) is None, p
        assert vals[5] in (0.0, 1.0), "6th value must be the magnet state"
    # one pick: approach/descend off, on at the part, carried on, off over the bin
    assert [float(p.split(",")[7]) for p in moves[1:9]] == [0, 0, 1, 1, 1, 1, 0, 0]
    assert float(moves[-1].split(",")[7]) == 0, "magnet must be off when parked"
    # the still image never changes, so every pick "fails"; after MAX_ATTEMPTS the
    # nearest part is skipped and the next one is tried
    assert s.stats["failed"] == 3
    assert any("failed" in r for r in s.skip_cells.values())


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as e:                       # noqa: BLE001
                failures += 1
                print(f"FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if failures else 0)
