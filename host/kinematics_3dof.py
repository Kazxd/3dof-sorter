
import math
from dataclasses import dataclass


# Geometry in millimetres
BASE_HEIGHT = 50.0
BASE_TO_SHOULDER = 30.0
L1 = 120.0
FOREARM = 80.0
MAGNET_OFFSET = 20.0

SHOULDER_Z = BASE_HEIGHT + BASE_TO_SHOULDER
L2 = FOREARM + MAGNET_OFFSET

# PCA9685 channel mapping
BASE_CHANNEL = 15
SHOULDER_CHANNEL = 14
ELBOW_CHANNEL = 13

# Provisional software limits.
# Replace these with measured limits before autonomous operation.
JOINT_LIMITS = {
    "base": (10.0, 170.0),
    "shoulder": (20.0, 160.0),
    "elbow": (20.0, 160.0),
}


@dataclass
class IKResult:
    base: float
    shoulder: float
    elbow: float
    reachable: bool
    message: str


def in_limits(name, angle):
    low, high = JOINT_LIMITS[name]
    return low <= angle <= high


def inverse_kinematics(x, y, z, elbow_up=True):
    """
    x: forward distance from base axis, mm
    y: lateral distance; positive means left, mm
    z: height above the table, mm

    Assumptions:
    - Base command 90 points forward.
    - Shoulder command 90 means horizontal upper arm.
    - Elbow command 90 means straight links.
    - Increasing shoulder raises the upper arm.
    - Increasing elbow bends the forearm upward.
    - Magnet offset is along the forearm.
    """

    # Base rotation
    base = 90.0 + math.degrees(math.atan2(y, x))

    # Target relative to shoulder pivot
    r = math.hypot(x, y)
    h = z - SHOULDER_Z

    # Distance from shoulder pivot to target
    d2 = r * r + h * h
    d = math.sqrt(d2)

    # Check geometric reachability
    if d > L1 + L2 or d < abs(L1 - L2):
        return IKResult(
            base, 0.0, 0.0, False,
            "Target outside arm reach"
        )

    # Relative elbow bend: 0 degrees means straight links
    cos_q2 = (d2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))

    q2_abs = math.acos(cos_q2)
    q2 = q2_abs if elbow_up else -q2_abs

    # Upper-arm angle relative to horizontal
    alpha = math.atan2(h, r) - math.atan2(
        L2 * math.sin(q2),
        L1 + L2 * math.cos(q2)
    )

    # Convert physical angles to servo commands
    shoulder = 90.0 + math.degrees(alpha)
    elbow = 90.0 + math.degrees(q2)

    # Validate all three commands
    if not all([
        in_limits("base", base),
        in_limits("shoulder", shoulder),
        in_limits("elbow", elbow),
    ]):
        return IKResult(
            base, shoulder, elbow, False,
            "Geometrically reachable, but outside configured servo limits"
        )

    return IKResult(
        base, shoulder, elbow, True,
        "Target reachable within configured limits"
    )


if __name__ == "__main__":
    # Example target: 150 mm forward, 0 mm sideways,
    # 50 mm above the table.
    target = (150.0, 0.0, 50.0)

    for elbow_up in (True, False):
        result = inverse_kinematics(
            *target, elbow_up=elbow_up
        )
        print("\nElbow-up solution:", elbow_up)
        print(result)
