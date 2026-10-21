"""Three-DOF inverse/forward kinematics for a base-yaw + 2-link arm.

Compatibility: preserves the public API used by sorter.py and arm_console.py:
solve(), reachable(), forward_kinematics(), joint_heights(), kin_to_servo(),
servo_to_kin(), limit_violation(), IKSolution and Unreachable.

PHYSICAL ASSUMPTION (IMPORTANT): the magnet must sit on a passive gravity-leveling
pivot/gimbal. The magnet face remains vertical while the forearm changes angle.
TOOL_DOWN is therefore treated as a WORLD-VERTICAL offset, not an offset fixed to
the forearm. A rigidly bolted magnet does not satisfy this model.
"""
from dataclasses import dataclass
import math
import config as C

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll")


class Unreachable(Exception):
    """No solution satisfies reach, calibrated servo limits and safety floors."""


@dataclass
class IKSolution:
    kin: tuple       # five slots: base, shoulder, elbow, inactive wrist pitch, inactive roll
    servo: tuple     # five servo commands; magnet state is still appended by the caller
    pitch: float     # -90 degrees: face-down, provided the passive gimbal is installed


def wrap_deg(angle, period=360.0):
    """Wrap into [-period/2, period/2)."""
    return (angle + period / 2.0) % period - period / 2.0


def kin_to_servo(i, kin_deg):
    joint = C.JOINTS[JOINT_NAMES[i]]
    return joint["zero"] + joint["direction"] * kin_deg


def servo_to_kin(i, servo_deg):
    joint = C.JOINTS[JOINT_NAMES[i]]
    return (servo_deg - joint["zero"]) / joint["direction"]


def limit_violation(i, servo_deg):
    """Return a reason if a commanded servo angle exceeds its configured limits."""
    name = JOINT_NAMES[i]
    joint = C.JOINTS[name]
    if servo_deg < joint["min"] - 1e-6:
        return f"{name} needs servo {servo_deg:.1f} deg < min {joint['min']:.1f}"
    if servo_deg > joint["max"] + 1e-6:
        return f"{name} needs servo {servo_deg:.1f} deg > max {joint['max']:.1f}"
    return None


def _angles_for_target(r, z):
    """Return candidate (shoulder, relative elbow) angles for a target radius/height."""
    l2 = float(C.L2)
    l3 = float(C.L3)
    # The wrist/pivot axis is directly above the magnet face by TOOL_DOWN mm.
    rw = r - float(getattr(C, "SHOULDER_OFFSET", 0.0))
    zw = z + float(C.TOOL_DOWN) - float(C.L1)
    d2 = rw * rw + zw * zw
    denom = 2.0 * l2 * l3
    if denom <= 0:
        raise Unreachable("L2 and L3 must be positive")
    cos_elbow = (d2 - l2 * l2 - l3 * l3) / denom
    if cos_elbow > 1.0 + 1e-9:
        raise Unreachable(f"too far (shoulder-to-pivot distance {math.sqrt(d2):.1f} mm)")
    if cos_elbow < -1.0 - 1e-9:
        raise Unreachable(f"too close (shoulder-to-pivot distance {math.sqrt(d2):.1f} mm)")
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    alpha = math.acos(cos_elbow)

    # Try positive relative elbow angle first (the user's elbow servo increases
    # when bending upward), then the opposite branch if configured limits require it.
    candidates = []
    for t3 in (alpha, -alpha):
        t2 = math.atan2(zw, rw) - math.atan2(l3 * math.sin(t3), l2 + l3 * math.cos(t3))
        candidates.append((math.degrees(t2), math.degrees(t3)))
    return candidates


def forward_kinematics(kin):
    """Kinematic angles -> (x, y, z, pitch) of magnet contact point in robot frame."""
    t1, t2, t3 = (math.radians(float(kin[i])) for i in range(3))
    l1, l2, l3 = float(C.L1), float(C.L2), float(C.L3)
    r = float(getattr(C, "SHOULDER_OFFSET", 0.0)) + l2 * math.cos(t2) + l3 * math.cos(t2 + t3)
    pivot_z = l1 + l2 * math.sin(t2) + l3 * math.sin(t2 + t3)
    # With a gravity-leveling gimbal, the tool offset is vertical in the world frame.
    z = pivot_z - float(C.TOOL_DOWN)
    pitch = -90.0
    return r * math.cos(t1), r * math.sin(t1), z, pitch


def joint_heights(kin):
    """Return elbow-axis and forearm-end pivot heights above the table."""
    t2 = math.radians(float(kin[1]))
    t3 = math.radians(float(kin[2]))
    elbow_z = float(C.L1) + float(C.L2) * math.sin(t2)
    pivot_z = elbow_z + float(C.L3) * math.sin(t2 + t3)
    return elbow_z, pivot_z


def solve(x, y, z, closing_axis_deg=None, pitches=None):
    """Solve a Cartesian magnet-face target using only base, shoulder and elbow.

    This does not command wrist pitch or roll. The serial interface still carries
    five servo slots, so those two values remain at their configured fixed zeros.
    """
    if not bool(getattr(C, "TOOL_SELF_LEVELING", False)):
        raise Unreachable(
            "3-DOF solver requires a passive gravity-leveling magnet mount; "
            "a rigid magnet needs a working wrist-pitch joint or a different model"
        )
    if abs(float(getattr(C, "TOOL_FORWARD", 0.0))) > 1e-6:
        raise Unreachable("3-DOF gimbal model assumes TOOL_FORWARD = 0; measure/reposition the mount")

    x, y, z = float(x), float(y), float(z)
    r = math.hypot(x, y)
    if z < float(C.MIN_TCP_Z):
        raise Unreachable(f"target z={z:.1f} mm is below table floor {C.MIN_TCP_Z:.1f} mm")

    t1 = math.degrees(math.atan2(y, x)) if r > 1e-6 else 0.0
    base_servo = kin_to_servo(0, t1)
    why = limit_violation(0, base_servo)
    if why:
        raise Unreachable(f"target outside base swing ({why})")

    failures = []
    try:
        candidates = _angles_for_target(r, z)
    except Unreachable as exc:
        raise Unreachable(f"target ({x:.1f}, {y:.1f}, {z:.1f}) mm: {exc}") from exc

    for t2, t3 in candidates:
        # No active wrist pitch or roll: kinematic angle 0 maps to their fixed servo zero.
        kin = (t1, t2, t3, 0.0, 0.0)
        servo = tuple(kin_to_servo(i, kin[i]) for i in range(5))
        bad = next((limit_violation(i, servo[i]) for i in range(5)
                    if limit_violation(i, servo[i]) is not None), None)
        if bad:
            failures.append(bad)
            continue

        elbow_z, pivot_z = joint_heights(kin)
        if elbow_z < float(C.MIN_ELBOW_Z):
            failures.append(f"elbow axis would be too low ({elbow_z:.1f} mm)")
            continue
        if pivot_z < float(C.MIN_WRIST_Z):
            failures.append(f"forearm-end pivot would be too low ({pivot_z:.1f} mm)")
            continue

        # Round magnet: the planar angle of the part does not change the command.
        return IKSolution(kin=kin, servo=servo, pitch=-90.0)

    detail = failures[0] if failures else "no valid elbow branch"
    raise Unreachable(f"no safe pose for ({x:.1f}, {y:.1f}, {z:.1f}) mm: {detail}")


def reachable(x, y, z, closing_axis_deg=None):
    try:
        solve(x, y, z, closing_axis_deg)
        return True
    except Unreachable:
        return False
