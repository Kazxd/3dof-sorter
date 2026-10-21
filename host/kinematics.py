"""Three-DOF IK/FK for a base-yaw + shoulder/elbow arm with a rigid magnet.

The Cartesian target is the centre of the magnet's flat contact face. L2 is the
shoulder-to-elbow distance and L3 is the elbow-axis-to-contact-face-centre distance
measured along the forearm. The magnet has no independently controlled wrist pitch
or roll: its actual pitch follows the forearm angle plus TOOL_AXIS_DEG.
"""
from dataclasses import dataclass
import math
import config as C

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll")


class Unreachable(Exception):
    """No solution satisfies geometric reach, servo limits, and safety checks."""


@dataclass
class IKSolution:
    kin: tuple       # five slots: base, shoulder, elbow, fixed wrist pitch, fixed roll
    servo: tuple     # five servo commands; magnet state is appended by the caller
    pitch: float     # actual rigid-tool pitch; -90 deg means face-down


def wrap_deg(angle, period=360.0):
    """Wrap an angle into [-period/2, period/2)."""
    return (angle + period / 2.0) % period - period / 2.0


def kin_to_servo(i, kin_deg):
    joint = C.JOINTS[JOINT_NAMES[i]]
    return joint["zero"] + joint["direction"] * kin_deg


def servo_to_kin(i, servo_deg):
    joint = C.JOINTS[JOINT_NAMES[i]]
    return (servo_deg - joint["zero"]) / joint["direction"]


def limit_violation(i, servo_deg):
    """Return a reason if a command exceeds the configured servo limits."""
    name = JOINT_NAMES[i]
    joint = C.JOINTS[name]
    if servo_deg < joint["min"] - 1e-6:
        return f"{name} needs servo {servo_deg:.1f} deg < min {joint['min']:.1f}"
    if servo_deg > joint["max"] + 1e-6:
        return f"{name} needs servo {servo_deg:.1f} deg > max {joint['max']:.1f}"
    return None


def _angles_for_target(r, z):
    """Return candidate (shoulder, relative-elbow) angles for the magnet centre."""
    l2 = float(C.L2)
    l3 = float(C.L3)
    rw = r - float(getattr(C, "SHOULDER_OFFSET", 0.0))
    # Unlike the old gimbal model, the rigid magnet centre is the distal endpoint
    # of L3; there is no separate world-vertical TOOL_DOWN offset.
    zw = z - float(C.L1)
    d2 = rw * rw + zw * zw
    denom = 2.0 * l2 * l3
    if denom <= 0:
        raise Unreachable("L2 and L3 must be positive")
    cos_elbow = (d2 - l2 * l2 - l3 * l3) / denom
    if cos_elbow > 1.0 + 1e-9:
        raise Unreachable(f"too far (shoulder-to-magnet distance {math.sqrt(d2):.1f} mm)")
    if cos_elbow < -1.0 - 1e-9:
        raise Unreachable(f"too close (shoulder-to-magnet distance {math.sqrt(d2):.1f} mm)")
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    alpha = math.acos(cos_elbow)

    candidates = []
    for t3 in (alpha, -alpha):
        t2 = math.atan2(zw, rw) - math.atan2(
            l3 * math.sin(t3), l2 + l3 * math.cos(t3)
        )
        candidates.append((math.degrees(t2), math.degrees(t3)))
    return candidates


def forward_kinematics(kin):
    """Return (x, y, z, actual tool pitch) for the rigid magnet contact centre."""
    t1, t2, t3 = (math.radians(float(kin[i])) for i in range(3))
    l1, l2, l3 = float(C.L1), float(C.L2), float(C.L3)
    r = float(getattr(C, "SHOULDER_OFFSET", 0.0)) + l2 * math.cos(t2) + l3 * math.cos(t2 + t3)
    z = l1 + l2 * math.sin(t2) + l3 * math.sin(t2 + t3)
    pitch = wrap_deg(math.degrees(t2 + t3) + float(getattr(C, "TOOL_AXIS_DEG", -90.0)))
    return r * math.cos(t1), r * math.sin(t1), z, pitch


def joint_heights(kin):
    """Return elbow-axis height and magnet contact-centre height above the table."""
    t2 = math.radians(float(kin[1]))
    t3 = math.radians(float(kin[2]))
    elbow_z = float(C.L1) + float(C.L2) * math.sin(t2)
    tool_z = elbow_z + float(C.L3) * math.sin(t2 + t3)
    return elbow_z, tool_z


def _pitch_targets(pitches):
    values = pitches if pitches is not None else getattr(C, "PITCH_CANDIDATES", [-90.0])
    if isinstance(values, (int, float)):
        values = [values]
    values = [float(v) for v in values]
    return values or [float(getattr(C, "TOOL_AXIS_DEG", -90.0))]


def solve(x, y, z, closing_axis_deg=None, pitches=None):
    """Solve base/shoulder/elbow IK for the rigid magnet contact-centre target.

    The actual magnet pitch is derived from shoulder + elbow motion. With ``pitches``
    omitted, candidate branches are ranked by closeness to the configured preferred
    pitch. When a caller explicitly supplies ``pitches``, the selected pose must be
    within TOOL_PITCH_TOLERANCE_DEG of one of those requested pitches; otherwise it is
    rejected. ``closing_axis_deg`` is retained for API compatibility but ignored: a
    round electromagnet has no jaw-closing axis and there is no wrist-roll actuator.
    """
    x, y, z = float(x), float(y), float(z)
    r = math.hypot(x, y)
    if z < float(C.MIN_TCP_Z):
        raise Unreachable(f"target z={z:.1f} mm is below table floor {C.MIN_TCP_Z:.1f} mm")

    t1 = math.degrees(math.atan2(y, x)) if r > 1e-6 else 0.0
    base_servo = kin_to_servo(0, t1)
    why = limit_violation(0, base_servo)
    if why:
        raise Unreachable(f"target outside base swing ({why})")

    try:
        candidates = _angles_for_target(r, z)
    except Unreachable as exc:
        raise Unreachable(f"target ({x:.1f}, {y:.1f}, {z:.1f}) mm: {exc}") from exc

    targets = _pitch_targets(pitches)
    strict_pitch = pitches is not None
    tolerance = float(getattr(C, "TOOL_PITCH_TOLERANCE_DEG", 25.0))
    valid = []
    failures = []
    pitch_rejections = []

    for t2, t3 in candidates:
        kin = (t1, t2, t3, 0.0, 0.0)
        servo = tuple(kin_to_servo(i, kin[i]) for i in range(5))
        bad = next((limit_violation(i, servo[i]) for i in range(5)
                    if limit_violation(i, servo[i]) is not None), None)
        if bad:
            failures.append(bad)
            continue

        elbow_z, _tool_z = joint_heights(kin)
        if elbow_z < float(C.MIN_ELBOW_Z):
            failures.append(f"elbow axis would be too low ({elbow_z:.1f} mm)")
            continue
        # There is no separate wrist pivot on this 3-DOF rigid-tool model. The distal
        # endpoint is the magnet contact centre and is intentionally allowed near the table.
        _fx, _fy, _fz, actual_pitch = forward_kinematics(kin)
        pitch_error = min(abs(wrap_deg(actual_pitch - target)) for target in targets)
        if strict_pitch and pitch_error > tolerance:
            pitch_rejections.append((actual_pitch, pitch_error))
            continue
        # Prefer the most downward-facing branch; for ties, prefer more elbow clearance.
        valid.append((pitch_error, -elbow_z, kin, servo, actual_pitch))

    if not valid:
        if pitch_rejections:
            pitch, error = min(pitch_rejections, key=lambda row: row[1])
            requested = ", ".join(f"{v:.1f}" for v in targets)
            raise Unreachable(
                f"rigid magnet pitch {pitch:.1f} deg is {error:.1f} deg from requested "
                f"pitch(es) [{requested}]; tolerance is {tolerance:.1f} deg"
            )
        detail = failures[0] if failures else "no valid elbow branch"
        raise Unreachable(f"no safe pose for ({x:.1f}, {y:.1f}, {z:.1f}) mm: {detail}")

    _error, _clearance, kin, servo, pitch = min(valid, key=lambda row: (row[0], row[1]))
    return IKSolution(kin=kin, servo=servo, pitch=pitch)


def reachable(x, y, z, closing_axis_deg=None):
    try:
        solve(x, y, z, closing_axis_deg)
        return True
    except Unreachable:
        return False
