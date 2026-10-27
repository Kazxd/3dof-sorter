> **Setup guide:** [Full hardware, calibration, testing, and dataset setup guide](README_robot_arm_sorter.md)
> Includes end-to-end hardware/firmware setup, camera calibration, tests, and YOLO dataset acquisition.

# Vision-guided 5-DOF electromagnetic fastener sorter

A phone camera looks down at a mat. The PC finds washers, nuts and bolts, works out
where each one is in millimetres, solves the arm's joint angles, and sends them over
USB to an Arduino Nano or ESP32. The board moves five servos smoothly through a PCA9685
and switches a 12 V electromagnet through a MOSFET. Each part is lifted by the magnet
and dropped into its own bin.

```
 phone (IP Webcam) ──Wi-Fi──▶ PC: camera.py ─▶ vision.py ─▶ mapping.py ─▶ kinematics.py ─▶ arm_link.py
                                   newest       YOLO + HSV     pixels → mm      5-DOF IK         USB serial
                                   frame        fallback       (homography)     + limits         <M,seq,…*CS>
                                                                                                   │
        5 servos ◀── PCA9685 ◀── I²C ◀── Nano / ESP32: firmware/arm_controller.ino ◀──────────────┘
        magnet   ◀── MOSFET  ◀── D7  ◀── parse · clamp · min-jerk smoothing · magnet safety · ACK/DONE
```

`host/sorter.py` runs the cycle: **park → scan → select → plan → press + magnet on → lift →
bin → magnet off → park → verify**. The camera only looks while the arm is parked out of
view, so the arm never hides parts or gets mistaken for one.

The earlier 3-DOF vision dashboard lives on the
[`3dof-dashboard`](https://github.com/Kazxd/3dof-sorter/tree/3dof-dashboard) branch.

## Bench status (27 Oct 2026)

What is actually built and what has been run on the real arm. For the report, this is the
ground truth; everything below it describes the full design.

**The arm on the bench is a 3-joint arm** (base, shoulder, elbow; the kit's wrist and
gripper were removed) driven by an ESP32 and a PCA9685, with the KK-P20/15 magnet
swinging on the printed hanger from `cad/magnet_hanger.scad` and switched by an **L298N**
(not a MOSFET). Everything is measured and tuned in `host/config.py`:

| Item | Value | How it was found |
|---|---|---|
| L1 table -> shoulder axis | 95 mm | ruler: 62 mm to the turntable top + 33 mm |
| L2 shoulder -> elbow axes | 120 mm | ruler |
| L3 elbow axis -> hanger pegs | 75 mm | ruler |
| Magnet face below the pegs | 51 mm | 61 mm on the CAD, corrected on the bench (`TOOL_DOWN`) |
| Elbow "straight" | servo 123, bigger = forearm down | jogging with `servo_test.ino` and `arm_console.py` |
| Shoulder | servo 90 = level, 180 = just past vertical | same |
| Home / park | base 90, shoulder 170, elbow 90 (upright, weight over the base) | chosen so power-up cannot tip the arm |
| Pick strip on the table | 177-185 mm from the base axis, +/-30 mm sideways | computed from the IK with the limits above |
| Bins | three spots on a 178 mm radius, reached by turning the base | same |

**Verified on hardware:** the whole motion side. `demo.py` connects over USB, homes,
descends onto a nut at 185 mm, switches the magnet on, lifts, swings the base to the bin,
lowers, releases, returns home and repeats for a bolt, with the phone feed live in the
window (video of 27 Oct). All 29 host tests pass, the firmware compiles.

**Open issue, the only one:** the magnet grips the nut and then **lets go as the shoulder
starts lifting**, and does not hold a flat bolt at all. The timing points to the 12 V
supply sagging under the servo load (one adapter feeds both the buck for the servos and the
magnet) plus the ~2 V the L298N drops. Not yet tried: separate supplies (magnet on the
adapter, servos on the battery, common GND) and a relay/MOSFET instead of the L298N. The
checks are listed under *Known limitations*.

**Not yet run on hardware:** the vision loop (`sorter.py` end to end) and the camera
calibration. The camera stream itself works (`CAMERA_URL`, IP Webcam at 640x480).

**Power-up order that works:** servo 6 V on first, hold the arm, then plug in the USB. The
firmware wakes the servos one at a time, so nothing whips.

## Folder layout

| Path | What it does |
|---|---|
| `host/config.py` | **Every number you measure or tune.** Link lengths, servo zeros, pick heights, bins, HSV, thresholds. |
| `host/sorter.py` | Main program and state machine. |
| `host/camera.py` | Background frame grabber: always the newest frame, auto-reconnect. |
| `host/vision.py` | YOLO detector, HSV + contour fallback, fusion, pick point. |
| `host/mapping.py` | Pixel ↔ mm homography, optional ArUco live recalibration. |
| `host/kinematics.py` | Closed-form 5-DOF IK and FK, joint limits, table-collision floor. |
| `host/arm_link.py` | Serial protocol (seq numbers, checksum, retries, ACK/DONE) and a simulator. |
| `host/get_model.py` | Downloads public Roboflow datasets, merges them into washer/nut/bolt, trains YOLOv8. |
| `host/collect_images.py` | Captures training photos from your own rig. |
| `host/calibrate_camera.py` | Click or ArUco calibration → `calibration/homography.json`. |
| `host/hsv_tuner.py` | Sliders for the HSV fallback; shows how each blob is classified. |
| `host/arm_console.py` | Jog servos, switch the magnet, test IK with `go x y z`. |
| `host/motion_test.py` | Staged movement test: every joint, magnet, IK points, speed cap, repeatability. |
| `host/demo.py` | Scripted pick-and-place of parts at fixed spots with the live camera window; `--calibrate` lets the arm calibrate the camera. |
| `host/tests/` | Hardware-free tests and a synthetic scene generator. |
| `firmware/arm_controller/` | Board firmware. |
| `firmware/servo_test/` | Standalone first power-up sketch: drive servos from the Serial Monitor. |
| `cad/` | Printable electromagnet housing: STLs, parametric OpenSCAD source, assembly guide. |

## Hardware and wiring

| From | To |
|---|---|
| PCA9685 VCC / GND | ESP32 3V3 / GND (logic power only) |
| PCA9685 SDA / SCL | ESP32 GPIO21 / GPIO22 (Nano: A4 / A5) |
| PCA9685 V+ (screw terminal) | **6 V from the buck converter** (12 V adapter -> buck), at least 5 A |
| PCA9685 ch 15, 14, 13 | base, shoulder, elbow (ch 12, 11 are the unused wrist slots) |
| L298N IN1 | ESP32 GPIO18 (D18, left pin row); IN2 -> GND; ENA bridged with a wire |
| L298N 12V / GND | 12 V adapter, GND shared with the ESP32; 5V-EN jumper on |
| L298N OUT1 / OUT2 | Magnet leads (the board has its own flyback diodes) |

The L298N drops about 2 V, so the magnet sees ~10 V. A relay module or a logic-level
MOSFET module (gate via 220 ohm, 10 k to GND, 1N5819 across the coil) gives it the full 12 V.

Join every ground: servo supply, 12 V supply and the board. Add a 1000 µF capacitor
across V+ and GND on the PCA9685. Never power the servos from the Nano's 5 V pin.
The full magnet wiring diagram is in [cad/README.md](cad/README.md#wiring).

**End effector.** The gripper and its servo come off. A KK-P20/15 magnet (12 V,
about 0.2 A, 20 x 15 mm) hangs from the printed hanger (`cad/magnet_hanger.scad`,
`stl/magnet_hanger_petg.stl`) on two 4 mm pegs in holes drilled through the forearm. It
swings freely, so the face always points down whatever the forearm does; the IK aims the
pegs and places the face `TOOL_DOWN` below them (`TOOL_MOUNT = "hanging"`). The older
spring-loaded housing (`TOOL_MOUNT = "rigid"`) is still in `cad/` for arms with a wrist.

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
Module) → Upload. The Serial Monitor at 115200 shows `<READY,ARM5MAG,2>`. Close the
Serial Monitor before running the Python tools; only one program can hold the port.

## Getting a model

**Quickest:** follow [TRAINING.md](TRAINING.md). It is one click in Google Colab, takes about an hour on a free GPU, and needs no photos of our own.

```bash
set ROBOFLOW_API_KEY=your_key
python get_model.py download
python get_model.py merge
python get_model.py train --device 0
```

`set` is for Windows `cmd`. In PowerShell use `$env:ROBOFLOW_API_KEY="your_key"`.

* **download** pulls 11 public Roboflow datasets and one synthetic Hugging Face set. All
  were checked by hand for a top-down view of loose parts, about 17k images in total.
  Highlights: [0428parts](https://universe.roboflow.com/new-workspace-jdteu/0428parts) (3.7k),
  [mechanical parts](https://universe.roboflow.com/abdalrahman-gamal/mechanical-parts-detection-glf33) (5.2k),
  [ECEN250 hardware](https://universe.roboflow.com/ecen250finalproject/hardware-g8ena) (1.4k) and
  [yellow_silver](https://universe.roboflow.com/jingjing-huang/yellow_silver) (1.4k). The full
  list with licences is at the top of `get_model.py`. It uses each dataset's newest version
  and caps big ones at 1,500 training images, so no single source dominates.
* **merge** renames every class containing washer, nut, bolt or screw to our three labels and
  drops the rest (bearings, gears, numbered classes).
* **train** fine-tunes `yolov8n.pt` and copies the result to `host/models/best.pt`.

**Then add your own photos.** This matters more than anything else on the list. Run
`python collect_images.py` and take 250–400 frames on the real mat. Label them `washer`,
`nut`, `bolt`, export as YOLOv8, unzip into `datasets/own/`, then run `merge` and `train` again.

Without `best.pt` the sorter still runs on the HSV fallback alone.

## Calibration checklist

1. **Print and fit the magnet housing.** Follow [cad/README.md](cad/README.md). Test every
   part you want to sort with a fridge magnet: stainless will not lift.

2. **Servo pulses and home** (firmware). Power the servos with the horns *off*. The board
   sends every servo to `HOME_POSE` (90°). Fit the horns so the arm stands straight up.
   If a servo buzzes at its end stop, narrow `PULSE_MIN_US` / `PULSE_MAX_US`.

3. **Joint zeros and directions** → `JOINTS`. In `python arm_console.py`, move one joint at
   a time with `j <i> <deg>` until it sits at its kinematic zero. The zero positions are:
   * base: arm pointing straight forward (+X)
   * shoulder: upper arm horizontal
   * elbow and wrist pitch: in line with the previous link

   That servo angle is `zero`. Now raise the servo angle a little. The positive directions
   are: base turns left (counter-clockwise from above), shoulder lifts, elbow and wrist
   bend upward. If the joint moved that way, `direction = +1`; if not, `-1`. Set `min` /
   `max` where nothing collides, and copy them into `LIMIT_MIN` / `LIMIT_MAX` in the firmware.

4. **Link lengths and magnet offset** → `L1`–`L3`, `TOOL_MOUNT`, `TOOL_FORWARD`, `TOOL_DOWN`.
   Measure axis centre to axis centre. L1 is table to shoulder axis. For the magnet head,
   measure from the wrist pitch axis, as shown in the install guide.

5. **Check the IK and the magnet.** In `arm_console.py`, run `go 160 0 30`, then
   `go 200 60 30`, then `go 140 -80 30`, and measure where the magnet face really is.
   Errors above 5 mm mean step 3 or 4 is off. Then put a nut under the magnet, use
   `go x y 2`, then `mag 1`, lift with `go x y 60`, and finally `mag 0`.

6. **Camera mount.** Put the phone on the tripod, lens parallel to the table. Lock focus,
   exposure and white balance in IP Webcam, and keep a 4:3 resolution.

7. **Homography** → `CALIB_POINTS_MM`, then `python calibrate_camera.py`. Mark 4–9 points
   on the mat, measure them from the base axis, click them in order, and check the grid with
   `--verify`. For bump-proof calibration use ArUco markers (`USE_ARUCO = True`).

8. **Pick area** → `WORKSPACE_X/Y`. `python sorter.py --no-arm` marks unreachable parts in red.

9. **HSV fallback** → `HSV_*`. A green cutting mat under silver parts works best. Tune with
   `python hsv_tuner.py` and paste the printed lines into `config.py`.

10. **Pick heights, park and bins** → `PICK`, `PARK_SERVO`, `BINS`. `PICK[...]["z"]` is the
    height of the part's top face. Park must hold the arm outside the camera view. Bins go
    outside the pick area; `arm_console.py bins` visits each one.

11. **Dry run.** `python sorter.py --sim`, then `python sorter.py`. Keep a hand near the
    power switch for the first runs.

## Testing the arm's movement

Two programs, used in this order on a newly built arm:

1. **`firmware/servo_test/servo_test.ino`**: flash it, open the Serial Monitor at 115200
   ("Newline"), and drive one servo at a time. `0`–`4` selects a servo, `+` / `-` steps it,
   `w` sweeps it, `h` centres everything, `m` pulses the magnet, `o` switches all outputs off.
   Every servo stays limp until you command it. Use it to fit the horns, check wiring and
   note each joint's zero and safe limits.
2. **`host/motion_test.py`**: flash `arm_controller.ino` again, then run
   `python motion_test.py`. It walks through link → home → each joint → magnet → IK points
   → speed cap → repeatability → park. It asks you to confirm what you see at each stage,
   tells you which `config.py` value to change when something is off, and saves a report to
   `runs/`. Useful options: `--sim` (no hardware), `--stages joints,ik`, `--amp 10`
   (smaller swings). Ctrl+C stops the arm and brings it home slowly.

## Running

```bash
python sorter.py                    # full system
python sorter.py --no-arm           # vision + reachability preview, nothing moves
python sorter.py --sim              # everything except the real arm
python sorter.py --no-yolo          # force the HSV fallback
python demo.py --port auto          # scripted picks at the spots in demo.PARTS, live camera window
python demo.py --port auto --calibrate   # the arm parks the magnet at 6 known spots; click it each time
```

`demo.py` is what ran on the bench. The parts sit at measured spots (no detection), but every
move is planned by the same `plan()` as the sorter, so it exercises IK, limits, the protocol
and the magnet sequence exactly. `--port auto` finds the USB port; `SERIAL_PORT` in
`config.py` is the ESP32's Wi-Fi address for the TCP mode.

Window keys: `q` parks, then quits. `space` pauses. `r` retries skipped spots.
Each run writes `runs/picks_<time>.csv` with label, position, source, result and cycle
time per pick.

## How a pick works

| Step | Pose | Magnet |
|---|---|---|
| approach | `APPROACH_HEIGHT` (15 mm) above the part, magnet hanging face-down | off |
| descend | 1 mm below the part's top (`PRESS_DEPTH`): just touching, a deeper press tilts the hanging magnet | off |
| magnet on | hold for 700 ms while the coil current builds | **on** |
| lift | `LIFT_HEIGHT` up, slowly (`DESCEND_MS`) | on |
| to bin, lower | above the bin, then down to drop height | on |
| magnet off | hold for 400 ms so residual magnetism lets go | **off** |
| retreat, park | up, then out of the camera's view | off |

The next scan checks the result. If the part is still there (not steel, or picked up off
centre), it is retried once and then skipped.

## How the pieces work

**Vision fusion.** YOLO at confidence ≥ `CONF_HIGH` is trusted. Between `CONF_LOW` and
`CONF_HIGH`, it is only trusted if the outline's shape agrees: long and thin is a bolt,
round with a hole is a washer, hex with a hole is a nut. Anything YOLO missed or doubted
is classified by shape alone. Shape is measured in millimetres on the table plane, so a
tilted phone does not distort it. The pick point is the outline's centroid, shifted
slightly toward a bolt's heavier head. Each scan uses 5 frames, and a part must appear in
at least 3 of them.

**IK.** th1 = atan2(Y, X). With the hanging magnet (`TOOL_MOUNT = "hanging"`) the target
for the two-link solver is the hanger pivot, `TOOL_DOWN` straight above the magnet face;
the face is always level, so there is no pitch to solve. Shoulder and elbow come from the
law of cosines; both elbow branches are tried, and a branch is rejected if the forearm would
point up more than `HANG_MAX_FOREARM_UP_DEG` (the hanger would rest on it) or the elbow
would go below `MIN_ELBOW_Z`. The old rigid-mount model (face fixed to the forearm, pitch
follows it, `PITCH_CANDIDATES` / `TOOL_PITCH_TOLERANCE_DEG`) is kept behind
`TOOL_MOUNT = "rigid"`. Every solution is checked against the servo limits in `JOINTS`.

**Edge cases.**

| Situation | What happens |
|---|---|
| Part out of reach | Skipped, never attempted; shown in red with the reason. |
| Bin out of reach | The program refuses to start. |
| Camera drops | The arm stays parked and the grabber reconnects. |
| Lost ACK | Packet resent with the same seq; the board ignores duplicates. |
| Board stops answering | Stop, reconnect once, otherwise park and exit. |
| Non-magnetic part | Verify sees it still there; it is retried once, then skipped. |
| PC goes silent / magnet on too long | Firmware cut-offs (5 s silence, 30 s on) exist but are **off** (`MAGNET_AUTO_OFF 0`): they dropped parts during hand-paced tests. Only the PC switches the magnet off. Set it to 1 for unattended runs. |

## Serial protocol

`<PAYLOAD*CS>\n`, where CS is the XOR of the payload bytes in two hex digits.

| PC → board | Meaning |
|---|---|
| `M,seq,a1,a2,a3,a4,a5,mag,ms` | Move 5 servos (degrees), set magnet (1/0), over `ms` |
| `H,seq,ms` | Go to firmware `HOME_POSE`, magnet off |
| `S,seq` / `R,seq` | Stop and hold / relax (servo outputs and magnet off) |
| `Q,seq` / `P,seq` | Query pose / ping |

The board answers with `ACK,seq,actual_ms`, then `DONE,seq` when the move ends.
`ERR,seq,CODE` means the packet was rejected; `POS`, `PONG` and `WARN` cover the rest.
The magnet switches at the start of a move. The board stretches `ms` when needed so no
joint exceeds `MAX_SPEED_DPS`.

## Known limitations

* **Steel only.** Stainless, brass and aluminium fasteners will not lift.
* **Bolts.** A bolt lying flat touches the magnet along a line, so holding force is much
  lower than the 3 kg rating. It is still plenty for M3–M8 bolts.
* **Torque.** Hobby-servo arms are weakest at the shoulder when stretched out. If the arm
  sags at long reach, shrink `WORKSPACE_X` or slow `MAX_SPEED_DPS`.
* **Reach.** With the hanging magnet and the shoulder kept above ~15 deg below level, the
  face reaches the table only 175-190 mm from the base axis. Parts closer in are out of
  reach; the base rotation gives the width. The arm cannot pick on its left side: the
  shoulder tops out just past vertical, so the "left" is only the stable resting direction.
* **Magnet hold (open).** On the bench the nut is gripped and then released as the lift
  starts; a flat bolt is not held. To isolate it: `mag 1` in `arm_console.py`, press a nut
  on the face by hand, then `j 1 120`; if it drops as the arm moves, the 12 V is sagging
  under servo load: put the magnet and the servos on separate supplies (common GND). If it
  is weak even at rest, feed the magnet 12 V directly for a second; if that is clearly
  stronger, replace the L298N with a relay or MOSFET module. Pick bolts by the head.
* **Vision not yet run on the arm.** `sorter.py` is tested end to end against a synthetic
  scene, a simulated arm and a fake serial board; the camera stream works. Calibration
  (`demo.py --calibrate` or `calibrate_camera.py`) and a live sorting run are the next step.
