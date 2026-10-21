# Apply and test this patch

1. Back up your clone.
2. Copy the included `host/config.py`, `host/kinematics.py`, and `firmware/arm_controller/arm_controller.ino` to the matching paths in the repository after reviewing the diff.
3. Do not flash/run it on the physical arm yet unless you have a passive gravity-leveling magnet mount. The default `TOOL_SELF_LEVELING = False` intentionally blocks IK commands to prevent the program assuming a vertical magnet that is not physically vertical. After installing and verifying that mount, change it to `True`.
4. Flash the firmware with `CHANNEL={15,14,13,12,11}`; only channels 15/14/13 are driven. Channels 12/11 are explicitly turned off. The host still sends five servo values so `arm_link.py` remains protocol-compatible.
5. Run a syntax/import check from `host/`:

   ```bash
   python -m py_compile config.py kinematics.py arm_link.py sorter.py
   python tests/test_all.py
   ```

   The project's original tests may assert 5-DOF-specific behavior and may need to be updated for this alternate model.
6. With `TOOL_SELF_LEVELING = True` only after the physical mount is verified, first test calculations in `python arm_console.py --sim`. Then support the arm and test one point at a time with low speed and a hand near the power switch. Do not start the full live sorter as the first test.
7. The patch deliberately limits the initial pickup region to a tiny zone around X=188.5–190 mm and Y=±5 mm, with approach/lift height 15 mm and bins around radius 195 mm. This is a geometry smoke-test zone, not a practical final workspace. Expand it only after real FK measurements and joint-limit checks.
