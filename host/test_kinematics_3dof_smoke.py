"""Equation-only smoke test for the alternate 3-DOF kinematics; no hardware access."""
import config as C
import kinematics as K

# This test temporarily enables the math model in memory; it does not validate whether
# the physical magnet mount self-levels. Keep config.py's default False until verified.
old_flag = C.TOOL_SELF_LEVELING
C.TOOL_SELF_LEVELING = True
try:
    cases = {
        "washer_pick": (189.0, 0.0, C.PICK["washer"]["z"] - C.PRESS_DEPTH),
        "washer_approach": (189.0, 0.0, C.PICK["washer"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT),
        "nut_pick": (189.0, 0.0, C.PICK["nut"]["z"] - C.PRESS_DEPTH),
        "nut_approach": (189.0, 0.0, C.PICK["nut"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT),
        "bolt_pick": (189.0, 0.0, C.PICK["bolt"]["z"] - C.PRESS_DEPTH),
        "bolt_approach": (189.0, 0.0, C.PICK["bolt"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT),
    }
    cases.update({f"bin_{label}": xyz for label, xyz in C.BINS.items()})
    cases.update({f"bin_{label}_approach": (x, y, z + C.APPROACH_HEIGHT)
                  for label, (x, y, z) in C.BINS.items()})

    for name, xyz in cases.items():
        sol = K.solve(*xyz)
        actual = K.forward_kinematics(sol.kin)[:3]
        assert max(abs(a - b) for a, b in zip(actual, xyz)) < 1e-5, (name, xyz, actual)
        assert all(C.JOINTS[j]["min"] - 1e-6 <= sol.servo[i] <= C.JOINTS[j]["max"] + 1e-6
                   for i, j in enumerate(K.JOINT_NAMES))
        print(f"PASS {name}: {tuple(round(a, 1) for a in sol.servo)}")

    C.TOOL_SELF_LEVELING = False
    try:
        K.solve(189.0, 0.0, 0.0)
    except K.Unreachable:
        print("PASS self-leveling safety guard")
    else:
        raise AssertionError("IK should be blocked while TOOL_SELF_LEVELING=False")
finally:
    C.TOOL_SELF_LEVELING = old_flag
