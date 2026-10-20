"""
arm_console.py - jog the arm by hand to calibrate servos and test the IK.

    python arm_console.py            real arm (SERIAL_PORT in config.py)
    python arm_console.py --sim      no hardware

Index: 0 base, 1 shoulder, 2 elbow, 3 wrist pitch, 4 wrist roll (servo degrees), 5 magnet (1 on / 0 off).

  pose                     current 6-value pose (servo deg) + kinematic angles
  j <i> <deg> [ms]         move one servo                 e.g.  j 1 120
  n <i> <delta>            nudge one servo                e.g.  n 2 -5
  all a1 a2 a3 a4 a5 a6    move every servo at once
  mag 1 | mag 0            electromagnet on / off
  home | park | stop | relax
  fk                       where the tool point is for the current pose (mm)
  ik x y z                 solve only, no motion
  go x y z                 move the magnet face there (magnet state unchanged)
  bins                     hover over each bin in turn
  help | quit
"""
import argparse
import logging
import shlex

import config as C
import kinematics as K
from arm_link import ArmError, ArmLink, SimArm


def show_pose(pose):
    names = ("base", "shoulder", "elbow", "wrist_p", "wrist_r", "magnet")
    print("  servo deg : " + "  ".join(f"{n}={a:.1f}" for n, a in zip(names, pose)))
    kin = [K.servo_to_kin(i, pose[i]) for i in range(5)]
    print("  kinematic : " + "  ".join(f"θ{i + 1}={a:.1f}" for i, a in enumerate(kin)))
    x, y, z, pitch = K.forward_kinematics(kin)
    print(f"  tool point: X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm   pitch={pitch:.1f} deg")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true")
    p.add_argument("--port")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    arm = SimArm(speed=1.0) if args.sim else ArmLink(port=args.port)
    try:
        arm.connect()
    except ArmError as e:
        raise SystemExit(f"connect failed: {e}")
    try:
        pose = list(arm.query()[0])         # start from where the board says it is
    except ArmError:
        pose = list(C.HOME_SERVO)
    print(__doc__)
    show_pose(pose)

    while True:
        try:
            line = input("arm> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        try:
            parts = shlex.split(line)
        except ValueError:
            print("  could not parse that line")
            continue
        cmd, a = parts[0].lower(), parts[1:]
        try:
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print(__doc__)
            elif cmd == "pose":
                show_pose(pose)
            elif cmd == "j":
                i, deg = int(a[0]), float(a[1])
                ms = int(a[2]) if len(a) > 2 else 800
                pose[i] = deg
                arm.move(pose, ms)
                show_pose(pose)
            elif cmd == "n":
                i, d = int(a[0]), float(a[1])
                pose[i] += d
                arm.move(pose, 300)
                show_pose(pose)
            elif cmd == "all":
                pose = [float(v) for v in a[:6]]
                arm.move(pose, 1500)
            elif cmd == "mag":
                pose[5] = 1 if a[0] in ("1", "on") else 0
                arm.move(pose, C.MAGNET_SETTLE_MS)
            elif cmd == "home":
                arm.home()
                pose = list(C.HOME_SERVO)
            elif cmd == "park":
                pose = list(C.PARK_SERVO[:5]) + [pose[5]]
                arm.move(pose, C.PARK_MS)
            elif cmd == "stop":
                arm.stop()
            elif cmd == "relax":
                print("  WARNING: the arm will go limp. Hold it. Next move snaps to the target.")
                arm.relax()
            elif cmd == "fk":
                show_pose(pose)
            elif cmd in ("ik", "go"):
                x, y, z = (float(v) for v in a[:3])
                sol = K.solve(x, y, z)
                print(f"  pitch {sol.pitch:.0f}  kin {[round(v, 1) for v in sol.kin]}  "
                      f"servo {[round(v, 1) for v in sol.servo]}")
                if cmd == "go":
                    pose = list(sol.servo) + [pose[5]]
                    arm.move(pose, C.MOVE_MS)
            elif cmd == "bins":
                for label, (bx, by, bz) in C.BINS.items():
                    sol = K.solve(bx, by, bz + C.APPROACH_HEIGHT)
                    print(f"  {label}: hovering at ({bx}, {by}, {bz + C.APPROACH_HEIGHT})")
                    pose = list(sol.servo) + [pose[5]]
                    arm.move(pose, C.MOVE_MS)
                    input("  enter for next bin ")
            else:
                print("  unknown command - type help")
        except (IndexError, ValueError):
            print("  missing or bad number - type help")
        except K.Unreachable as e:
            print(f"  unreachable: {e}")
        except ArmError as e:
            print(f"  arm error: {e}")
    arm.close()


if __name__ == "__main__":
    main()
