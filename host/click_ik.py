"""
click_ik.py - click a point on the IP camera feed, get the 3-DOF IK for it.

    python click_ik.py                    dry run: show IK only, nothing moves
    python click_ik.py --sim              also "move" a simulated arm with g
    python click_ik.py --move             really move the arm with g
    python click_ik.py --camera 0 --z 40

Needs calibration/homography.json (python calibrate_camera.py first).
Put this file in host/ next to kinematics_3dof.py.

Keys:  left click = choose target     g = go to target (needs --move / --sim)
       z / x = target height +5 / -5 mm     h = home     q / Esc = quit
Servo packets: [base, shoulder, elbow, 90, 90, magnet] -> PCA9685 ch 15, 14, 13.
"""
import argparse
import time

import cv2

import config as C
import kinematics as K
from arm_link import ArmError, ArmLink, SimArm
from camera import FrameGrabber
from mapping import PlaneMapper

WIN = "click_ik"
state = {"click": None, "mouse": (-1, -1)}


def on_mouse(event, x, y, flags, param):
    state["mouse"] = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        state["click"] = (x, y)


def solve_any(x, y, z):
    """Use the main rigid-mount IK solver shared with sorter.py."""
    try:
        return K.solve(x, y, z), []
    except K.Unreachable as exc:
        return None, [str(exc)]


def put(img, s, row, color=(255, 255, 255)):
    p = (10, 24 + 22 * row)
    cv2.putText(img, s, p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, p, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera")
    ap.add_argument("--port")
    ap.add_argument("--z", type=float, default=40.0, help="target height above table, mm")
    ap.add_argument("--move", action="store_true", help="drive the real arm")
    ap.add_argument("--sim", action="store_true", help="simulated arm")
    ap.add_argument("--ms", type=int, default=2000, help="move duration")
    args = ap.parse_args()

    if not C.HOMOGRAPHY_FILE.exists():
        raise SystemExit("No calibration. Run: python calibrate_camera.py")
    mapper = PlaneMapper.load()

    arm = None
    if args.sim:
        arm = SimArm(speed=1.0)
    elif args.move:
        arm = ArmLink(port=args.port)
    if arm:
        try:
            arm.connect()
        except ArmError as e:
            raise SystemExit(f"arm connect failed: {e}")

    grab = FrameGrabber(args.camera)
    grab.start()
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)

    z = args.z
    target = None            # (x, y)
    result, msgs = None, []
    try:
        while True:
            frame, _ = grab.latest()
            if frame is None:
                if cv2.waitKey(50) & 0xFF in (ord("q"), 27):
                    break
                continue
            h, w = frame.shape[:2]
            if not mapper.set_frame_size(w, h):
                print("WARNING: stream aspect ratio differs from calibration")
            img = frame.copy()

            if state["click"]:
                u, v = state["click"]
                target = mapper.px_to_mm(u, v)
                state["click"] = None
            if target:
                result, msgs = solve_any(target[0], target[1], z)
                u, v = mapper.mm_to_px(*target)
                col = (0, 255, 0) if result else (0, 0, 255)
                cv2.drawMarker(img, (int(u), int(v)), col, cv2.MARKER_CROSS, 24, 2)

            mx, my = state["mouse"]
            if 0 <= mx < w and 0 <= my < h:
                cx, cy = mapper.px_to_mm(mx, my)
                put(img, f"cursor X={cx:6.1f} Y={cy:6.1f} mm", 0, (0, 255, 255))
            if target:
                put(img, f"target X={target[0]:.1f} Y={target[1]:.1f} Z={z:.0f} mm", 1)
                if result:
                    put(img, f"base {result.servo[0]:.1f}  shoulder {result.servo[1]:.1f}  "
                             f"elbow {result.servo[2]:.1f}", 2, (0, 255, 0))
                else:
                    put(img, msgs[0] if msgs else "unreachable", 2, (0, 0, 255))
            put(img, "click=target g=go z/x=height h=home q=quit" +
                ("" if arm else "   [DRY RUN]"), 3)
            cv2.imshow(WIN, img)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("z"):
                z += 5
            elif key == ord("x"):
                z = max(C.MIN_TCP_Z, z - 5)
            elif key == ord("h") and arm:
                arm.home()
            elif key == ord("g"):
                if not arm:
                    print("dry run: restart with --move or --sim to move")
                elif not result:
                    print("target not reachable:", msgs)
                else:
                    pose = list(result.servo) + [0]
                    print("moving to", [round(a, 1) for a in pose])
                    try:
                        arm.move(pose, args.ms, on_wait=lambda: cv2.waitKey(1))
                    except ArmError as e:
                        print("arm error:", e)
    finally:
        grab.stop()
        if arm:
            arm.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
