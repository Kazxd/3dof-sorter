# 3-DOF adaptation patch for `3dof-sorter`

This is a **starter patch**, not a complete copy of the repository. It contains a replacement `host/kinematics.py`, a `host/config.py` based on the supplied file, and a modified `firmware/arm_controller/arm_controller.ino`. `host/arm_link.py` is intentionally unchanged because the packet protocol can still carry five servo angles plus magnet state.

## Non-negotiable mechanical assumption

The replacement kinematics assumes a **passive gravity-leveling mount/gimbal**: the electromagnet face stays vertical and points down while the forearm angle changes. `TOOL_DOWN = 20` is modelled as a world-vertical offset from the forearm-end pivot to the magnet face. The repository's original rigid plate mount does not self-level on its own.

If your electromagnet is rigidly bolted to the forearm and follows its angle, **do not run the real sorter with this solver**. The magnet may approach parts at a tilt. Either add a passive self-leveling pivot, restore/repair a wrist-pitch servo and keep the original five-DOF solver, or develop and validate a rigid-tool planner with the resulting orientation/reach restrictions.

## Contents

- `host/config.py`: preliminary measurements/settings based on the conversation: table-to-shoulder 80 mm, shoulder-to-elbow 120 mm, elbow-to-tool pivot 80 mm, 20 mm hanging tool offset, servo zeros at 90 degrees, and a deliberately tiny initial workspace.
- `host/kinematics.py`: 3-DOF Cartesian position IK and FK. It preserves the API used by `sorter.py` / `arm_console.py`, returning five servo values so serial packets do not change. Wrist pitch and roll remain at 90 degrees.
- `firmware/arm_controller/arm_controller.ino`: maps the active joints to PCA9685 channels 15/14/13, uses a 1000-2000 us provisional pulse range (matching the user's tested ~205-410 tick range at 50 Hz), uses 60-120 degree provisional limits, and disables PWM on channels 12/11 while keeping the 5-angle packet layout.

## Before flashing or moving

1. Check wiring, especially PCA9685 channel numbers and the MOSFET wiring for the 12 V electromagnet. Keep common ground and a flyback diode across the coil.
2. Keep servo power off while fitting horns. Then test **one joint at a time** with the arm supported. The pulse range and 60-120 degree limits are provisional; do not widen them until physical stops/collisions have been checked.
3. Confirm your magnet can remain vertical without driving the wrist servos. Set `TOOL_SELF_LEVELING = True` only after this is physically true.
4. The workspace in this config is intentionally tiny because the initial joint range is conservative. First validate `forward_kinematics` / `solve` in simulation and the arm console; only widen the workspace after measuring repeatable real positions.
5. `PARK_SERVO` is deliberately set to the neutral pose as a bench-test placeholder; it is not guaranteed to hide the arm from the camera. Set a physically checked park pose before camera scanning.
6. Keep `PICK`, `BINS`, and the camera homography in robot-frame millimetres. The bin coordinates in this patch are provisional positions near a 195 mm radius; physically place the bins there or remeasure/reconfigure them.

## Apply

Back up your repository first. Copy these files into the corresponding paths in your clone, review the diffs, and run the repository's hardware-free tests. The original tests may include assumptions specific to the 5-DOF solver, so an existing failure is not proof of a hardware fault; adapt the IK tests to the three active joints and the gimbal model. Do not start with `python sorter.py` on the real arm. First use `--sim`, then test one waypoint at a time with the arm console and a hand near the power switch.
