# Vision-guided 5-DOF fastener sorter

A phone camera looks down at a mat. The PC finds washers, nuts and bolts, works out
where each one is in millimetres, solves the arm's joint angles, and sends them over
USB to an Arduino Nano or ESP32. The board drives six servos through a PCA9685 and
moves them smoothly. Each part is dropped into its own bin.

```
 phone (IP Webcam) ──Wi-Fi──▶ PC: camera.py ─▶ vision.py ─▶ mapping.py ─▶ kinematics.py ─▶ arm_link.py
                                   newest       YOLO + HSV     pixels → mm      5-DOF IK         USB serial
                                   frame        fallback       (homography)     + limits         <M,seq,…*CS>
                                                                                                   │
                         servos ◀── PCA9685 ◀── I²C ◀── Nano / ESP32: firmware/arm_controller.ino ◀┘
                                                         parse · clamp · min-jerk smoothing · ACK/DONE
```

`sorter.py` runs the cycle: **park → scan → select → plan → pick → place → park → verify**.
The camera only looks while the arm is parked out of view, so the arm never hides or
gets mistaken for a part.

## Folder layout

| Path | What it does |
|---|---|
| `host/config.py` | **Every number you measure or tune.** Link lengths, servo zeros, bins, HSV, thresholds. |
| `host/sorter.py` | Main program and state machine. |
| `host/camera.py` | Background frame grabber: always the newest frame, auto-reconnect. |
| `host/vision.py` | YOLO detector, HSV + contour fallback, fusion, pick point and long-axis angle. |
| `host/mapping.py` | Pixel ↔ mm homography, optional ArUco live recalibration. |
| `host/kinematics.py` | Closed-form 5-DOF IK and FK, joint limits, table-collision floor. |
| `host/arm_link.py` | Serial protocol (seq numbers, checksum, retries, ACK/DONE) and a simulator. |
| `host/get_model.py` | Downloads public Roboflow datasets, merges them into washer/nut/bolt, trains YOLOv8. |
| `host/collect_images.py` | Captures training photos from your own rig. |
| `host/calibrate_camera.py` | Click or ArUco calibration of the camera → `calibration/homography.json`. |
| `host/hsv_tuner.py` | Sliders for the HSV fallback, shows how each blob is classified. |
| `host/arm_console.py` | Jog servos, read kinematic angles, test IK with `go x y z`. |
| `host/tests/` | Hardware-free tests and a synthetic scene generator. |
| `firmware/arm_controller/arm_controller.ino` | Board firmware. |

## Hardware and wiring

| From | To |
|---|---|
| PCA9685 VCC / GND | Board 5V / GND (logic power only) |
| PCA9685 SDA / SCL | Nano A4 / A5, or ESP32 GPIO21 / GPIO22 |
| PCA9685 V+ (screw terminal) | **Separate 5–6 V servo supply, at least 5 A**. Join its GND to the board GND. |
| PCA9685 ch 0–5 | base, shoulder, elbow, wrist pitch, wrist roll, gripper |

Add a 1000 µF capacitor across V+ and GND on the PCA9685. Never power the servos from
the Nano's 5 V pin: the board browns out and resets mid-move.

Optional electromagnet instead of the gripper: logic-level MOSFET (IRLZ44N / AO3400)
gate on PCA9685 channel 5, 1N5819 flyback diode across the coil, coil on its own supply.
Set `GRIPPER_IS_MAGNET 1` in the firmware and `END_EFFECTOR = "magnet"` in `config.py`.

## Install

```bash
cd host
python -m venv .venv
.venv\Scripts\activate
pip install -r ../requirements.txt
python tests/test_all.py
```

Python 3.10+. All tests should print `PASS`. They need no camera, arm or YOLO.

Flash the firmware: Arduino IDE → Library Manager → install **Adafruit PWM Servo Driver
Library** → open `firmware/arm_controller/arm_controller.ino` → select Nano (or ESP32 Dev
Module) → Upload. Open the Serial Monitor at 115200 and you should see `<READY,ARM5,1>`.
Close the Serial Monitor before running the Python tools; only one program can hold the port.

## Getting a model

No downloadable YOLOv8 `.pt` for washer/nut/bolt is good enough to trust on your
table without retraining, so `get_model.py` builds one:

```bash
set ROBOFLOW_API_KEY=your_key
python get_model.py download
python get_model.py merge
python get_model.py train --device 0
```

`set` is for Windows `cmd`. In PowerShell use `$env:ROBOFLOW_API_KEY="your_key"`. The
free key is under roboflow.com → Settings → API Keys.

* **download** pulls [yellow_silver](https://universe.roboflow.com/jingjing-huang/yellow_silver)
  (about 1.4k top-down images with ArUco markers; silver and yellow bolts, nuts and washers)
  and [nutsnbolts](https://universe.roboflow.com/tktk-hodc3/nutsnbolts) (50 images: nut, bolt,
  washer). Check each page's Dataset tab for the newest version number and edit `SOURCES` if needed.
* **merge** renames every class containing washer, nut, bolt or screw to our three labels and
  drops anything else.
* **train** fine-tunes `yolov8n.pt` and copies the result to `host/models/best.pt`. On a
  laptop CPU this is slow. Google Colab's free GPU takes about 15–30 minutes.

**Then add your own photos.** This matters more than anything else on the list.
Run `python collect_images.py` and take 250–400 frames on the real mat. Label them in
Roboflow or CVAT with exactly `washer`, `nut`, `bolt`, export as YOLOv8, unzip into
`datasets/own/`, then run `merge` and `train` again. Your images are repeated 3× in
training so the model learns your setup first.

Without `best.pt` the sorter still runs on the HSV fallback alone (the status bar shows
`hsv-only`).

## Calibration checklist

Do these in order. Each step says which part of `config.py` it fills in.

1. **Servo pulses and home** (firmware). Power the servos with the horns *off*. The
   board sends every servo to `HOME_POSE` (90°). Fit the horns so the arm stands straight
   up ("candle pose"), with the gripper half open. If a servo buzzes at its end stop, narrow
   `PULSE_MIN_US` / `PULSE_MAX_US` for that channel.

2. **Joint zeros and directions** → `JOINTS`. Run `python arm_console.py`, then move one
   joint at a time with `j <i> <deg>` until it sits at its kinematic zero:
   * base: arm pointing straight forward (+X, away from the camera's near edge)
   * shoulder: upper arm horizontal, pointing forward
   * elbow and wrist pitch: in line with the previous link
   * wrist roll: jaws close sideways, across the arm
   
   That servo angle is `zero`. Now raise the servo angle a few degrees. The positive
   directions are: base turns left (counter-clockwise from above), shoulder lifts,
   elbow and wrist bend upward, roll turns counter-clockwise seen from above. If the joint
   moved that way, `direction = +1`; if not, `-1`. Then set `min` / `max` where nothing
   collides. Copy the same min/max into `LIMIT_MIN` / `LIMIT_MAX` in the firmware.

3. **Link lengths** → `L1`–`L4`, `SHOULDER_OFFSET`. Measure axis centre to axis centre
   with calipers. L1 runs from the table to the shoulder axis. L4 runs from the wrist pitch
   axis to the middle of the closed fingertips.

4. **Check the IK**. In `arm_console.py`, run `go 160 0 30`, then `go 200 60 30` and
   `go 140 -80 30`. Measure where the fingertips actually are with a ruler. Errors above
   5 mm mean step 2 or 3 is off. Use `fk` to see what the software thinks.

5. **Camera mount**. Put the phone on the tripod at its final height, lens parallel to the
   table. In IP Webcam, lock focus, exposure and white balance. Choose a 4:3 resolution
   such as 640×480 or 1280×960 and keep it from now on.

6. **Homography** → `CALIB_POINTS_MM`, then `python calibrate_camera.py`. Mark 4–9
   points on the mat, measure their X/Y from the base axis, enter them in `config.py`,
   then click them in order. The green grid should land on your ruler marks. Run
   `calibrate_camera.py --verify` and hover over marks to read their mm. For bump-proof
   calibration, print ArUco 4×4 markers (ids 0–3), set `ARUCO_MARKERS_MM` and `USE_ARUCO = True`.

7. **Pick area** → `WORKSPACE_X/Y`. Keep it inside what the arm reaches with the gripper
   pointing down. `python sorter.py --no-arm` marks unreachable parts in red.

8. **HSV fallback** → `HSV_*`. A green cutting mat under silver parts works best. Run
   `python hsv_tuner.py` and adjust until the mask is black on the mat and white on parts,
   with washer and nut holes visible. Press `p` and paste the printed lines into `config.py`.

9. **Park, bins and grasps** → `PARK_SERVO`, `BINS`, `GRASP`. Park must hold the whole arm
   outside the camera view. Bins must sit outside the pick area. `arm_console.py bins` visits
   each one. Set gripper `open` / `close` angles per class, and the grasp `z`, about half
   the part's height.

10. **Dry run**. `python sorter.py --sim` runs the real camera and vision while the arm
    motion is simulated and the packets are printed. Then run `python sorter.py`. Keep a
    hand near the servo power switch for the first runs.

## Running

```bash
python sorter.py                    # full system
python sorter.py --no-arm           # vision + reachability preview, nothing moves
python sorter.py --sim              # everything except the real arm
python sorter.py --no-yolo          # force the HSV fallback
```

Window keys: `q` parks, then quits. `space` pauses. `r` retries parts that were skipped.
Each run writes `runs/picks_<time>.csv` with label, position, source, result and cycle
time per pick. Use it for the pick success rate and cycle time metrics. Logs go to
`runs/sorter.log`.

Offline demo with no camera or arm:

```bash
python tests/synthetic.py
mkdir calibration
copy tests\out\homography.json calibration\homography.json
python sorter.py --camera tests/out/scene.png --sim --no-yolo
```

## How the pieces work

**Vision fusion.** YOLO at confidence ≥ `CONF_HIGH` is trusted. Between `CONF_LOW` and
`CONF_HIGH`, it is only trusted if the outline's shape agrees: long and thin is a bolt,
round with a hole is a washer, hex with a hole is a nut. Anything YOLO missed or doubted
is classified by shape alone. Shape numbers are measured on the outline mapped to
millimetres, so a tilted phone does not distort them. The pick point is the outline's
centroid, shifted slightly toward a bolt's head. Each scan uses 5 frames, and a part
must appear in at least 3 of them.

**IK.** θ1 = atan2(Y, X). The tool pitch is chosen next, pointing straight down (−90°)
if possible. The wrist centre is found by stepping back along the tool by L4. Shoulder and
elbow come from the law of cosines (elbow-up). θ4 = pitch − θ2 − θ3. θ5 turns the jaws
across a bolt's long axis. If −90° is out of reach or hits a joint limit, the solver tries
flatter or steeper pitches from `PITCH_CANDIDATES`. Every solution is checked against
servo limits and a table-height floor for the tool, wrist and elbow.

**Edge cases.**

| Situation | What happens |
|---|---|
| Part out of reach | Skipped, never attempted; shown in red with the reason. |
| Bin out of reach | The program refuses to start. |
| Camera drops | The arm stays parked and the grabber reconnects. |
| Stream resolution changes | Mapping rescales automatically; a different aspect ratio stops the scan. |
| Lost ACK | Packet resent with the same seq; the board ignores duplicates, so nothing moves twice. |
| Board stops answering | Stop, reconnect once, otherwise park and exit. |
| Pick fails | The next scan still sees the part, so it is retried. After 2 failures that spot is skipped. |
| Parts touching | Picked last. |
| Magnet mode | The firmware drops the magnet if the PC goes silent for 5 s. |

## Serial protocol

`<PAYLOAD*CS>\n`, where CS is the XOR of the payload bytes in two hex digits.

| PC → board | Meaning |
|---|---|
| `M,seq,a1,a2,a3,a4,a5,g,ms` | Move 5 joints + gripper (servo degrees) over `ms` |
| `H,seq,ms` | Go to firmware `HOME_POSE` |
| `S,seq` / `R,seq` | Stop and hold / relax (outputs off) |
| `Q,seq` / `P,seq` | Query pose / ping |

The board answers with `ACK,seq,actual_ms`, then `DONE,seq` when the move ends.
`ERR,seq,CODE` means the packet was rejected; `POS`, `PONG` and `WARN` cover the rest.
The board stretches `ms` when needed so no joint exceeds `MAX_SPEED_DPS`.

This differs from the suggested `<a1,a2,a3,a4,gripper>`:

* 5-DOF plus a gripper is six values, not five.
* The sequence number tells the PC which move finished and lets the board ignore repeats.
* The checksum rejects lines garbled on the wire.
* The duration lets the board smooth the motion.

For a Serial Monitor test, set `REQUIRE_CHECKSUM 0` (the default) and type
`<M,1,90,90,90,90,90,60,1500>`.

## Known limitations

* **Washers and a gripper.** A flat washer is 1–2 mm thick, so the fingertips must close
  almost on the table. Use thin, slightly compliant fingertip pads, set `GRASP["washer"]["z"]`
  around 0.5 mm, and expect a lower success rate than for nuts and bolts. The electromagnet
  option avoids this problem for steel parts.
* **Torque.** Hobby-servo arms are weakest at the shoulder when stretched out. If the arm
  sags at long reach, shrink `WORKSPACE_X`, slow `MAX_SPEED_DPS`, or fit a stronger shoulder servo.
* **Accuracy.** Expect ±3–6 mm from plastic gears and printed links. Approach always from
  the same hover point (the code does this), and use fingertip pads that tolerate a few mm.
* **Roll angle.** Roll is exact when the tool points straight down; it is approximate
  at other pitches.
* **Unverified on hardware.** The host code is tested against a synthetic scene and a
  simulated arm. The firmware has not been compiled or run here; expect to fix small
  things on first flash.
