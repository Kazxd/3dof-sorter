# Robot Arm Sorter — Setup, Calibration, and Test Guide

This repository contains a vision-guided fastener sorter using a 3-DOF arm (base, shoulder, elbow), an ESP32, a PCA9685 servo controller, a rigidly mounted electromagnet, and a camera mounted above the work surface.

> **Current status (22 October 2026): development / bench-test stage.** The ESP32 Wi-Fi protocol has connected successfully, the three active joints have moved in manual tests, YOLO weights were trained, and live detections were previewed. However, physical Cartesian accuracy is **not validated**: the IK test's PASS is based on operator confirmation, `PARK_SERVO` still equals `HOME_SERVO`, and a click in `click_ik.py` produced coordinates far from the intended target. Do **not** run autonomous physical sorting yet. Keep the electromagnet's 12 V supply disconnected while debugging arm motion.

## 1. System overview

- **Host:** Ubuntu Linux, Python 3.12 tested.
- **Controller:** ESP32 development board; Wi-Fi/TCP is the intended normal host link.
- **Servo controller:** PCA9685, I²C address `0x40`.
- **Active joints:** base, shoulder, elbow. Firmware keeps the wrist-pitch and wrist-roll protocol slots but disables their PWM outputs.
- **Tool:** rigidly mounted electromagnet. Tool pitch follows shoulder/elbow geometry; there is no active wrist pitch or roll.
- **Camera:** Android IP Webcam MJPEG stream, currently configured as `http://192.168.1.3:8080/video` (change this to the current phone IP).
- **Vision:** YOLO model at `host/models/best.pt`; HSV/contour fallback is available but isn't the focus of the current workflow.

### Robot coordinate convention

The coordinate origin is on the table directly below the base rotation axis:

- `+X`: straight out in front of the robot.
- `+Y`: to the robot's left when viewed from behind it.
- `+Z`: upward from the table.
- Distances are in millimetres; joint angles are in degrees.

## 2. Before powering the arm

1. Secure the arm to a stable base and remove loose objects from its travel area.
2. Put the arm mechanically near its configured home pose **before powering the servos/controller**. The firmware homes to 90° on the active servo channels at startup, so the arm may move when enabled.
3. Use a separate, appropriately rated servo supply for the servos; do not power the servos from the ESP32 3.3 V rail.
4. Keep the electromagnet's 12 V supply disconnected until its driver wiring, flyback diode, release behaviour, and safety checks have been independently verified.
5. Keep a hand near the servo-power switch during every physical test. Stop immediately if a joint binds, strains, collides, or moves unexpectedly.

## 3. Hardware and firmware

### 3.1 PCA9685 and servo wiring

The current firmware defines these PCA9685 channels:

| Joint slot | PCA9685 channel | Active in firmware? |
|---|---:|---|
| Base | 15 | Yes |
| Shoulder | 14 | Yes |
| Elbow | 13 | Yes |
| Wrist pitch | 12 | No; output disabled |
| Wrist roll | 11 | No; output disabled |

Current firmware parameters are provisional and must match the actual board/wiring:

- PCA9685 I²C address: `0x40`
- ESP32 SDA: GPIO 21
- ESP32 SCL: GPIO 22
- PWM frequency: 50 Hz
- Pulse range: 1000–2000 µs across 0–180°
- Active joint limits: base 5–175°, shoulder 10–160°, elbow 10–160°
- Home pose: `[90, 90, 90]` for the three active joints

The host joint limits in `host/config.py` and firmware limits must agree. Do not expand limits just to make an IK target reachable; verify mechanical clearance and calibrate joint zeros/directions first. Check the logic-voltage requirements of your PCA9685 breakout. ESP32 GPIOs are 3.3 V logic and are not 5 V tolerant.

Use a separate 5–6 V servo supply sized for the actual servo stall current (the project has used a supply rated at least 5 A as a starting point). Connect grounds according to the PCA9685/servo-supply manufacturer's instructions; the ESP32, servo supply, and any low-side MOSFET circuit need an appropriate common reference.

### 3.2 Electromagnet driver

The firmware uses ESP32 GPIO 18 (D18, on the left pin row) as a logic output for a suitable MOSFET driver. Never drive a coil directly from an ESP32 pin. Use a correctly rated logic-level MOSFET/module, gate resistor and pull-down as required, and a flyback diode across the coil (diode cathode/stripe toward the positive supply). The 12 V magnet supply is separate from the servo supply. Verify that the driver is wired for the configured active-high/active-low behaviour before connecting coil power.

Do not perform magnet tests while diagnosing IK or camera mapping. First validate positioning with the magnet power disconnected.

### 3.3 Install Arduino CLI and compile

Use the standalone CLI if the Snap-installed CLI fails due to AppArmor restrictions. On the development machine, the known working executable was `~/tools/arduino-cli/bin/arduino-cli`.

Install the ESP32 core and required library (if not already installed):

```bash
~/tools/arduino-cli/bin/arduino-cli core install esp32:esp32
~/tools/arduino-cli/bin/arduino-cli lib install "Adafruit PWM Servo Driver Library"
```

Compile from the repository root:

```bash
cd /path/to/robot_arm_sorter
~/tools/arduino-cli/bin/arduino-cli compile \
  --fqbn esp32:esp32:esp32 firmware/arm_controller
```

Upload only when the correct ESP32 board and USB port have been identified:

```bash
~/tools/arduino-cli/bin/arduino-cli board list
~/tools/arduino-cli/bin/arduino-cli upload \
  --fqbn esp32:esp32:esp32 \
  --port /dev/ttyACM0 \
  firmware/arm_controller
```

Replace `/dev/ttyACM0` with the actual port reported on your machine. Upload requires a USB connection; normal operation is intended to use Wi-Fi.

### 3.4 ESP32 Wi-Fi modes

`firmware/arm_controller/arm_controller.ino` supports a direct ESP32 access-point mode when `WIFI_SSID` is empty and a router/hotspot mode when router credentials are configured locally.

- Default AP name in the current firmware: `RobotArm-ESP32`.
- Default host endpoint: `tcp://192.168.4.1:8888`.
- In AP mode, connect the host to the ESP32 AP and use the password configured by `WIFI_AP_PASSWORD` in the firmware.
- For router mode, set `WIFI_SSID` and `WIFI_PASSWORD` locally, upload the firmware, read the ESP32 IP from its serial startup output, then set `SERIAL_PORT` in `host/config.py` to `tcp://<ESP32-IP>:8888`.

**Do not commit private Wi-Fi/router passwords.** If changing from AP mode to router mode, review `git diff` and ensure credentials are not staged. For integrated camera + arm operation, the host must be able to reach both the phone's camera stream and the ESP32 endpoint at the same time. The current camera URL is on `192.168.1.x`, whereas default ESP32 AP mode uses `192.168.4.1`; a single Wi-Fi interface usually cannot access both separate networks simultaneously. Prefer having the phone, host, and ESP32 on the same router/hotspot network, or use a separately configured network interface.

## 4. Python host environment

From the repository root, create and activate a virtual environment. Do not use `sudo pip` or install packages into Ubuntu's externally managed system Python.

```bash
cd /path/to/robot_arm_sorter
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip wheel
python -m pip install numpy opencv-python pyserial ultralytics pytest roboflow
```

`roboflow` is needed for the optional dataset-download workflow. PyTorch is installed as an Ultralytics dependency in the usual supported setup; verify that the installed PyTorch build can see CUDA if GPU training is intended.

Scripts are in `host/`. With the virtual environment activated:

```bash
cd host
python3 sorter.py --help
python3 click_ik.py --help
python3 calibrate_camera.py --help
python3 get_model.py --help
```

## 5. Configure the camera stream

1. Mount the Android phone firmly above the table, looking down at the work surface. Keep its pose fixed after calibration.
2. Start the IP Webcam server app on the phone and note the displayed IP address and port. Its common MJPEG endpoint is `http://<PHONE-IP>:8080/video`.
3. Put that URL in `host/config.py`:

   ```python
   CAMERA_URL = "http://<PHONE-IP>:8080/video"
   ```

4. Lock focus/exposure if the camera app supports it. Use good, even lighting and avoid changing the stream resolution after calibration.
5. Ensure the host can access the URL. If the camera cannot be opened, check the phone IP, port, URL path, firewall, and network before continuing.

The known calibrated stream size for this setup is **1920 × 1080**. If the stream resolution or aspect ratio changes, recalibrate and verify again.

## 6. Camera-to-table homography calibration

The calibration maps image pixels to table-plane millimetres. Accurate coordinates depend on physically measuring the reference points relative to the robot base axis; the numerical RMS alone does not prove calibration accuracy.

### 6.1 Mark and measure reference points

1. Identify the point on the table directly below the base rotation axis; this is `(0, 0)`.
2. Mark four or more well-separated points on the table. Measure each mark's X/Y coordinates with a ruler from the base reference and using the coordinate convention above.
3. Keep the marks flat on the same plane as the parts to be picked.
4. Update `CALIB_POINTS_MM` in `host/config.py` so that it contains the measured points **in the exact order you intend to click them**. The current configuration is:

   ```python
   CALIB_POINTS_MM = [
       (110.0,  110.0),
       (230.0,  110.0),
       (230.0, -110.0),
       (110.0, -110.0),
   ]
   ```

   Use those numbers only if they match your actual marked points. Do not use approximate labels as substitutes for physical measurements.

### 6.2 Capture the homography

```bash
cd /path/to/robot_arm_sorter/host
python3 calibrate_camera.py
```

In the window, click the centres of the measured marks in the configured order. The calibration tool supports:

- Left click: add the next reference point
- `u`: undo the last point
- `r`: restart the point sequence
- `f`: freeze/unfreeze the image
- `s`: save after enough points have been entered
- `q` or Escape: quit

The saved file should be `host/calibration/homography.json`. Check the path and image size printed by the program.

### 6.3 Verify, do not just trust RMS

```bash
python3 calibrate_camera.py --verify
```

Hover over each physical ruler mark and check that the displayed values match the measured coordinates. Then hover over at least one or two additional interior points whose coordinates you know. As a practical starting acceptance target, aim for approximately 5 mm or better at measured points across the work area. With exactly four correspondences, a homography can fit the four clicked points nearly perfectly by construction, so an RMS near zero is not proof that the camera mapping is physically correct.

Recalibrate whenever the phone/camera is moved, the lens/resolution/aspect ratio changes, or the table setup changes. Keep a backup of the known-good `homography.json` before running synthetic tests.

## 7. YOLO model and training

The inference weight path is `host/models/best.pt`. A trained model was present locally on 22 October 2026 (about 6 MB). The recorded public validation output reported mAP@50 0.993 and mAP@50–95 0.794; these are **not** proof of performance on the physical table. A separately labelled own-camera validation split was not present at the time of writing (`../datasets/own/data.yaml` was missing).

### 7.1 Preview YOLO on the live camera

The direct live inference preview can be tested without connecting to the arm by loading `models/best.pt` in an Ultralytics prediction script. Check washer, nut, and bolt individually, in multiple positions/orientations and under the actual lighting.

### 7.2 Dataset acquisition and retraining

The committed detector weights are at `host/models/best.pt`, so a fresh checkout can run inference without downloading the training images. The model is approximately 6.25 MB. Keep the weights and dataset acquisition as separate paths: users who only want to run the sorter do not need the full training corpus.

#### Obtain the training datasets

The repository's `train_colab.ipynb` is the current entry point for assembling the public fastener datasets. Open it in Google Colab and run the notebook from top to bottom. The notebook describes downloading 12 public datasets (approximately 17,000 top-down images), normalising the labels to `washer`, `nut`, and `bolt`, and merging them into a YOLO dataset at `datasets/merged/data.yaml`. The workflow uses Roboflow; obtain an API key from your Roboflow account when prompted. **Do not paste a personal API key into a committed notebook, Python file, README, or `.env` file.** Enter it only into the runtime's secret/input mechanism.

Two versioned source exports are present in the local dataset metadata:

- [yellow_silver, version 4](https://universe.roboflow.com/jingjing-huang/yellow_silver/dataset/4) — 3,641 images in its exported dataset metadata.
- [nutsnbolts, version 5](https://universe.roboflow.com/tktk-hodc3/nutsnbolts/dataset/5) — 130 images in its exported dataset metadata.

These are two identified sources, not a complete list of the 12. For an auditable rerun, preserve the exact source project and export version for every dataset in the notebook/downloader and record each source's licence and attribution. Do not substitute the latest version of a Roboflow project for a pinned export: a changed version may change both images and labels.

After dataset preparation, verify that `datasets/merged/data.yaml` names the expected three classes and that train/validation/test splits exist. Its generated `path` must resolve on the current machine; do not commit a machine-specific absolute path such as `/home/<user>/...`. The `.gitignore` intentionally excludes downloaded dataset contents so that Git clones stay small. The dataset workflow, source versions, licence notes, and checksums should be documented in `datasets/README.md` and, once complete, `datasets/manifest.yaml`.

#### Training configuration and reproduction status

The supplied `best.pt` checkpoint metadata records Ultralytics **8.4.160**, base model `yolov8n.pt`, `imgsz=640`, `epochs=80`, `batch=16`, `seed=0`, deterministic training enabled, and validation enabled. The notebook's visible example command currently uses `--epochs 40 --batch 32`; therefore, running that cell as currently written is **not an exact reproduction of this checkpoint's training configuration**. Align the notebook/downloader's training command and all dataset versions/splits before describing a new run as an exact reproduction. Preserve the generated run arguments, results, class mapping, and dataset manifest as experiment records; generated run directories remain ignored by Git.

The checkpoint's recorded public validation metrics were approximately precision 0.984, recall 0.981, mAP@50 0.993, and mAP@50–95 0.791. These are checkpoint metadata, not evidence of equivalent performance on your own camera/table. The own-camera dataset path `datasets/own/data.yaml` was absent in the recorded workspace; assess performance on a separate, labelled camera dataset before relying on live sorting.

#### Adding your own camera data

Record representative frames from the actual camera, label every visible washer, nut, and bolt in YOLO detection format, and split by recording session or video rather than randomly distributing adjacent near-identical frames across train and validation. That reduces leakage and gives a more meaningful estimate of performance on new scenes. Keep personal, private, or non-redistributable images out of the public repository unless their sharing terms explicitly allow it.

Before redistributing the datasets or weights, review the licences/attribution for all source datasets and the applicable Ultralytics licensing terms. The presence of a public download URL does not by itself guarantee permission to republish the downloaded images.
# Or run all stages: python3 get_model.py all --device 0
```

The script uses public fastener datasets and can incorporate an own dataset at `datasets/own/` (YOLO format, including `data.yaml`). To use video, extract representative frames, label each visible object as `washer`, `nut`, or `bolt` in a labelling tool, and export a YOLO detection dataset. Do not split near-identical consecutive frames randomly across train and validation; use separate clips/recording sessions where possible to better estimate generalization.

If distributing trained weights, check and include the relevant dataset/source attributions and licences. Keep raw datasets, API keys, local logs, and caches out of Git unless there is a deliberate reason to share them.

## 8. Tests and preview tools

### 8.1 Automated host tests

From `host/` with the virtual environment activated:

```bash
python3 -m pytest -q
```

The suite previously passed 15 tests, with one pytest collection warning for a helper class in `motion_test.py`. Re-run it after every code change; the reported result is historical, not a guarantee about the current checkout.

### 8.2 Synthetic scene

The synthetic test generator creates `host/tests/out/scene.png` and a matching synthetic homography. It is useful for offline tests, but its calibration is for a 960 × 720 synthetic image and must **never replace the real 1920 × 1080 camera calibration permanently**. Prefer the automated test suite, which constructs an isolated synthetic calibration for its end-to-end test. Back up the real calibration before any manual synthetic experiment and restore it immediately afterward.

### 8.3 Vision and IK preview

```bash
cd host
python3 sorter.py --help
python3 sorter.py --no-arm
```

`--no-arm` is for vision/IK preview and should not issue physical movement commands. The configured workspace controls which objects are considered. The current `WORKSPACE_X=(188.5, 190.0)` and `WORKSPACE_Y=(-5.0, 5.0)` values are intentionally a very narrow temporary test region, not the full calibrated table. Only widen the production workspace after the geometry and physical reach have been validated. `--sim` uses a simulated arm; it does not validate physical picking.

### 8.4 Interactive IK tool

```bash
python3 click_ik.py              # dry run; no real movement
python3 click_ik.py --help
```

The tool uses left click to select a table target, `z`/`x` to change target height by 5 mm, `g` to move (only when started with `--move` or `--sim`), `h` to home, and `q`/Escape to quit.

**Known issue:** a physical-mode test intended for `(215, 10) mm` instead selected `(130.6, -95.8) mm`, then later sent poses that did not match the intended point. This may relate to display/window scaling and mouse coordinates, or another target-selection issue. Fix and verify click-to-pixel mapping before using `click_ik.py --move` again.

## 9. Motion test and physical commissioning

Run the staged test only after reviewing the hardware, with the arm area clear and the servo-power switch immediately accessible:

```bash
cd host
python3 motion_test.py
```

Do not use `--yes` for a physical commissioning test; follow each prompt and say no/stop if the observed motion is not as described. A test result saying that IK points were reached may rely on manual confirmation; it is not an independent Cartesian measurement. Measure the actual magnet-face location and height at each target with a ruler, and only record IK as verified when the physical measurements agree.

Current known commissioning issues:

1. `PARK_SERVO` currently equals `HOME_SERVO` (`[90, 90, 90, 90, 90, 0]`), and the latest motion report failed the park check. Choose a distinct park pose only after visually checking a safe collision-free path; do not guess a folded pose.
2. The latest physical IK test looked like very small movements to the operator. The current workspace was only 1.5 mm wide in X and 10 mm in Y, so its six test targets were nearly identical; that explains repeated poses but does not explain why the first move from home looked small.
3. The solver's forward-kinematics self-check returns the target mathematically by construction; it does not compare the real arm to the target.
4. Current geometry values in `host/config.py` are `L1=90 mm`, `L2=120 mm`, `L3=100 mm`. L2 was estimated from a photo and should be measured axis-centre to axis-centre. L3 must match the elbow axis to the centre of the magnet's contact face along the forearm.
5. The rigid magnet has no independent wrist-pitch control. Some reachable positions yield a tilted magnet face; check the reported tool pitch and physical face angle before any contact move.
6. Confirm PCA9685 channels 15/14/13, servo pulse limits, joint directions, zero offsets, and the host/firmware limits all match the actual hardware.

### Required gate before automatic sorting

- [ ] The camera calibration is checked at corners and interior points with known physical coordinates.
- [ ] Mouse clicks in `click_ik.py` produce the intended table coordinates.
- [ ] Geometry and servo zeros/directions are physically measured or calibrated.
- [ ] Several broad, safely elevated Cartesian targets are measured and agree with prediction within the chosen tolerance.
- [ ] A collision-free `PARK_SERVO` is chosen and the park test passes.
- [ ] The magnet remains off while checking motion; later, magnet switching/release is tested separately with suitable safety precautions.
- [ ] A single manually selected pick is validated before autonomous cycles are enabled.

Do not run the live physical sorter until these checks are complete.

## 10. Git hygiene and committing changes

Before staging, review changes and avoid committing temporary backups/debug outputs:

```bash
cd /path/to/robot_arm_sorter
git status --short
git diff --check
git diff
```

Do not blindly use `git add -A`: this working tree has included `.bak` files, firmware debugging artifacts (`debug.cfg`, `debug_custom.json`, `esp32.svd`), patch-helper scripts, generated logs, model files, and datasets. Stage source files intentionally, review the staged diff, and make a commit only after tests pass.

Potential source files for the current implementation include:

```bash
git add \
  firmware/arm_controller/arm_controller.ino \
  host/arm_link.py \
  host/config.py \
  host/kinematics.py \
  host/kinematics_3dof.py \
  host/test_kinematics_3dof_smoke.py \
  host/tests/test_all.py \
  host/click_ik.py \
  README.md
```

If this guide is saved under another filename, substitute that filename for `README.md`. Before committing, inspect and confirm only the intended files are staged:

```bash
git diff --cached --stat
git diff --cached
git status --short
```

The trained `host/models/best.pt` is optional to commit. It makes fresh clones easier to run, but first check the dataset licences/attributions and intended repository size. If you choose to version it, explicitly stage it only after that review:

```bash
git add host/models/best.pt
```

Do not stage Wi-Fi/API credentials, private datasets, temporary `.bak*` files, generated run logs, or debugging artifacts unless you deliberately intend to maintain them in the repository.

After reviewing the staged diff:

```bash
git commit -m "Document ESP32 sorter setup and camera calibration"
git status --short
```

A clean working tree after the commit confirms the selected changes were committed; push only when you are ready to publish the changes to the configured remote.
