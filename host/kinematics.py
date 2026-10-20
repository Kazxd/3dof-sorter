"""
kinematics.py - closed-form geometric IK / FK for a 5-DOF arm.

Joint order everywhere: 0 base yaw, 1 shoulder pitch, 2 elbow pitch,
3 wrist pitch, 4 wrist roll. (The magnet, index 5 in a pose, is not a joint here.)

Kinematic angle conventions (degrees, see the diagram in README.md):
    θ1 base      0 = arm points along +X; positive = counter-clockwise seen from above.
    θ2 shoulder  angle of L2 above horizontal; 0 = horizontal forward, 90 = straight up.
    θ3 elbow     angle of L3 relative to L2; 0 = in line, negative = bent downwards.
    θ4 wrist     angle of the last link (wrist -> magnet head) relative to L3; 0 = in line.
    ψ  last-link direction = θ2 + θ3 + θ4
    φ  tool pitch = direction the magnet faces = ψ + TOOL_AXIS_DEG ; -90 = straight down.
       ("plate" mount: TOOL_AXIS_DEG = -90, so the magnet faces down when the plate is level.)
    θ5 roll      the magnet is round, so the sorter keeps roll at 0. solve() can still
                 turn it (closing_axis_deg) if a directional tool is ever fitted.

How the 5-DOF problem is made closed-form:
    1. θ1 comes straight from the target's bearing: atan2(Y, X).
    2. The arm is now planar (r, z). Choose a tool pitch φ (try -90 first),
       step back from the target by the magnet head's offset -> wrist centre.
    3. Shoulder + elbow reach the wrist centre: a 2-link triangle, solved
       with the law of cosines (elbow-up branch).
    4. θ4 = φ - θ2 - θ3 makes the tool point at φ.
    5. θ5 (wrist roll) stays at 0 for the round magnet.
    If a pitch fails (out of reach, joint limit, would hit the table) the next
    pitch in config.PITCH_CANDIDATES is tried.

Servo mapping: servo_deg = zero + direction * kinematic_deg (config.JOINTS).
"""
import math
from dataclasses import dataclass

import config as C

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_pitch", "wrist_roll")


class Unreachable(Exception):
    """No joint solution satisfies reach, joint limits and the safety floor."""


@dataclass
class IKSolution:
    kin: tuple        # 5 kinematic angles, degrees
    servo: tuple      # 5 servo angles, degrees (what gets sent to the board)
    pitch: float      # tool pitch that was used, degrees


# -----------------------------------------------------------------------------
# helpers
# -----------------------------------------------------------------------------
def wrap_deg(angle, period=360.0):
    """Wrap into [-period/2, period/2)."""
    return (angle + period / 2.0) % period - period / 2.0


def kin_to_servo(i, kin_deg):
    j = C.JOINTS[JOINT_NAMES[i]]
    return j["zero"] + j["direction"] * kin_deg


def servo_to_kin(i, servo_deg):
    j = C.JOINTS[JOINT_NAMES[i]]
    return (servo_deg - j["zero"]) / j["direction"]


def limit_violation(i, servo_deg):
    """Return a human-readable reason if a servo angle is outside its limits, else None."""
    name = JOINT_NAMES[i]
    j = C.JOINTS[name]
    if servo_deg < j["min"] - 1e-6:
        return f"{name} needs servo {servo_deg:.0f} deg < min {j['min']:.0f}"
    if servo_deg > j["max"] + 1e-6:
        return f"{name} needs servo {servo_deg:.0f} deg > max {j['max']:.0f}"
    return None


def _tool_offset(psi):
    """Wrist pitch axis -> magnet face, in the arm's (r, z) plane, for last-link angle psi (rad).
    TOOL_FORWARD runs along the last link, TOOL_DOWN runs 90 deg below it."""
    return (C.TOOL_FORWARD * math.cos(psi) + C.TOOL_DOWN * math.sin(psi),
            C.TOOL_FORWARD * math.sin(psi) - C.TOOL_DOWN * math.cos(psi))


# -----------------------------------------------------------------------------
# forward kinematics (used for checks, tests and the console)
# -----------------------------------------------------------------------------
def forward_kinematics(kin):
    """Kinematic angles -> (x, y, z, pitch) of the tool point, robot frame."""
    t1, t2, t3, t4 = (math.radians(a) for a in kin[:4])
    a3 = t2 + t3
    psi = a3 + t4
    tr, tz = _tool_offset(psi)
    r = C.SHOULDER_OFFSET + C.L2 * math.cos(t2) + C.L3 * math.cos(a3) + tr
    z = C.L1 + C.L2 * math.sin(t2) + C.L3 * math.sin(a3) + tz
    return r * math.cos(t1), r * math.sin(t1), z, math.degrees(psi) + C.TOOL_AXIS_DEG


def joint_heights(kin):
    """Heights above the table of the elbow axis and the wrist axis."""
    t2 = math.radians(kin[1])
    a3 = t2 + math.radians(kin[2])
    elbow_z = C.L1 + C.L2 * math.sin(t2)
    wrist_z = elbow_z + C.L3 * math.sin(a3)
    return elbow_z, wrist_z


# -----------------------------------------------------------------------------
# inverse kinematics
# -----------------------------------------------------------------------------
def _solve_planar(r, z, pitch_deg):
    """Shoulder / elbow / wrist angles that put the tool point at (r, z) with tool pitch φ."""
    psi = math.radians(pitch_deg - C.TOOL_AXIS_DEG)          # last-link angle for this pitch
    tr, tz = _tool_offset(psi)
    # wrist centre = target minus the magnet head's offset
    rw = r - C.SHOULDER_OFFSET - tr
    zw = z - C.L1 - tz
    d2 = rw * rw + zw * zw

    cos_t3 = (d2 - C.L2 ** 2 - C.L3 ** 2) / (2.0 * C.L2 * C.L3)
    if cos_t3 > 1.0:
        raise Unreachable("too far")
    if cos_t3 < -1.0:
        raise Unreachable("too close")
    t3 = -math.acos(cos_t3)                                   # elbow-up branch
    t2 = math.atan2(zw, rw) - math.atan2(C.L3 * math.sin(t3), C.L2 + C.L3 * math.cos(t3))
    t4 = psi - t2 - t3
    return math.degrees(t2), math.degrees(t3), wrap_deg(math.degrees(t4))


def _roll_options(t5):
    """Jaws are symmetric, so roll repeats every 180 deg: offer every equivalent angle."""
    base = wrap_deg(t5, 180.0)
    return sorted((base, base + 180.0, base - 180.0), key=abs)


def solve(x, y, z, closing_axis_deg=None, pitches=None):
    """
    Target (x, y, z) in mm, robot frame -> IKSolution.

    closing_axis_deg : direction on the table (robot frame, 0 = +X, CCW positive)
                       along which the jaws should close. None = don't care (roll 0).
    pitches          : tool pitches to try, default config.PITCH_CANDIDATES.

    Raises Unreachable with the most useful reason if nothing works.
    """
    r = math.hypot(x, y)
    t1 = math.degrees(math.atan2(y, x)) if r > 1e-6 else 0.0

    base_servo = kin_to_servo(0, t1)
    why = limit_violation(0, base_servo)
    if why:
        raise Unreachable(f"target is outside the base's swing ({why})")
    if z < C.MIN_TCP_Z:
        raise Unreachable(f"target z={z:.1f} is below the table floor {C.MIN_TCP_Z}")

    if closing_axis_deg is None:
        roll_choices = [0.0]
    else:
        # closing direction on the table = θ1 + 90 + θ5 when the tool points down
        roll_choices = _roll_options(closing_axis_deg - (t1 + 90.0))

    reasons = []
    for pitch in (pitches or C.PITCH_CANDIDATES):
        try:
            t2, t3, t4 = _solve_planar(r, z, pitch)
        except Unreachable as e:
            reasons.append(str(e))
            continue

        partial = (t1, t2, t3, t4)
        bad = None
        for i in (1, 2, 3):
            bad = limit_violation(i, kin_to_servo(i, partial[i]))
            if bad:
                break
        if bad:
            reasons.append(bad)
            continue

        elbow_z, wrist_z = joint_heights(partial)
        if elbow_z < C.MIN_ELBOW_Z or wrist_z < C.MIN_WRIST_Z:
            reasons.append("elbow or wrist would hit the table")
            continue

        for t5 in roll_choices:
            s5 = kin_to_servo(4, t5)
            if limit_violation(4, s5) is None:
                kin = (t1, t2, t3, t4, t5)
                servo = tuple(kin_to_servo(i, kin[i]) for i in range(5))
                return IKSolution(kin=kin, servo=servo, pitch=float(pitch))
        reasons.append(limit_violation(4, kin_to_servo(4, roll_choices[0])))

    if reasons and all(rs == "too far" for rs in reasons):
        raise Unreachable(f"out of reach: too far (r={r:.0f} mm, z={z:.0f} mm)")
    if reasons and all(rs == "too close" for rs in reasons):
        raise Unreachable(f"out of reach: too close to the base (r={r:.0f} mm)")
    detail = next((rs for rs in reasons if rs not in ("too far", "too close")), "no valid pitch")
    raise Unreachable(f"no pose for ({x:.0f}, {y:.0f}, {z:.0f}): {detail}")


def reachable(x, y, z, closing_axis_deg=None):
    try:
        solve(x, y, z, closing_axis_deg)
        return True
    except Unreachable:
        return False
