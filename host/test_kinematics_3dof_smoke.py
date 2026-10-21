"""Smoke tests for the rigid-mount 3-DOF arm. No hardware access."""

import config as C
import kinematics as K


def test_rigid_mount_pick_and_bin_targets_round_trip():
    # The hardware has a rigid magnet, not a passive leveling gimbal.
    assert C.TOOL_SELF_LEVELING is False

    cases = {
        "washer_pick": (
            189.0, 0.0,
            C.PICK["washer"]["z"] - C.PRESS_DEPTH,
        ),
        "washer_approach": (
            189.0, 0.0,
            C.PICK["washer"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT,
        ),
        "nut_pick": (
            189.0, 0.0,
            C.PICK["nut"]["z"] - C.PRESS_DEPTH,
        ),
        "nut_approach": (
            189.0, 0.0,
            C.PICK["nut"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT,
        ),
        "bolt_pick": (
            189.0, 0.0,
            C.PICK["bolt"]["z"] - C.PRESS_DEPTH,
        ),
        "bolt_approach": (
            189.0, 0.0,
            C.PICK["bolt"]["z"] - C.PRESS_DEPTH + C.APPROACH_HEIGHT,
        ),
    }

    cases.update({f"bin_{label}": xyz for label, xyz in C.BINS.items()})
    cases.update({
        f"bin_{label}_approach": (x, y, z + C.APPROACH_HEIGHT)
        for label, (x, y, z) in C.BINS.items()
    })

    for name, xyz in cases.items():
        sol = K.solve(*xyz)
        actual = K.forward_kinematics(sol.kin)

        assert max(abs(a - b) for a, b in zip(actual[:3], xyz)) < 1e-5, (
            name, xyz, actual
        )
        assert abs(actual[3] - sol.pitch) < 1e-6, (name, actual[3], sol.pitch)

        for i, joint_name in enumerate(K.JOINT_NAMES):
            reason = K.limit_violation(i, sol.servo[i])
            assert reason is None, (name, joint_name, reason)

        # Wrist pitch and roll are not actuated by this 3-DOF arm.
        assert sol.servo[3] == C.JOINTS["wrist_pitch"]["zero"]
        assert sol.servo[4] == C.JOINTS["wrist_roll"]["zero"]
