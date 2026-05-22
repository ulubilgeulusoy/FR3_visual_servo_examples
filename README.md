# FR3 Visual Servo Examples

Visual servoing for the Franka Research 3 (FR3) using ViSP, `libfranka`, Intel RealSense, and a Qt5 operator GUI.

This version of the project builds a Qt-based `servoFrankaIBVS_combined` application. ViSP handles camera acquisition, AprilTag pose estimation, and the IBVS control law, while Qt owns the operator-facing window, controls, and status display.

## What It Does

The app detects an AprilTag in the camera image and commands Cartesian camera-frame velocity so the robot approaches a desired viewing pose.

Current capabilities:
- single-tag servo mode (`--mode 1`)
- sequenced multi-tag mode (`--mode 2`) using target IDs `{1, 2, 1}` on a 5 s cycle
- adjustable desired standoff distance
- lost-target recovery with backoff and biased turning
- controller-side safety supervision for joint margins, workspace bounds, proximity, and Franka contact/collision/error conditions
- manual recovery pose through the GUI
- local `arm_moving` state POSTs to `http://127.0.0.1:8765/state`

## Requirements

- FR3 reachable over the network and ready for velocity control
- `libfranka`
- ViSP built with Franka, RealSense2, and pugixml support
- Intel RealSense SDK 2.x
- Qt5 development packages: `Widgets`, `Core`, `Gui`
- CMake >= 3.10
- C++17 compiler
- camera-to-end-effector calibration file for `--eMc`
- printed AprilTag with known physical size

If this is a new robot/camera setup, first complete calibration and verify the upstream beginner example works on your hardware:

https://github.com/yiherngang/Franka-Research-3-with-ROS-2-Imvia-lab/tree/main?tab=readme-ov-file#70-visual-servoing-with-franka-research-3

## Build

```bash
mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DViSP_DIR=$HOME/visp_install/lib/cmake/visp
cmake --build build -j"$(nproc)" --target servoFrankaIBVS_combined
```

This version builds the Qt-based `servoFrankaIBVS_combined` target from:
- `src/main_qt.cpp`
- `src/MainWindow.cpp`
- `src/VisualServoController.cpp`

## Run

Direct binary:

```bash
./build/servoFrankaIBVS_combined \
  --eMc config/eMc.yaml \
  --ip 172.16.0.2 \
  --tag-size 0.05 \
  --desired-factor 8 \
  --adaptive-gain \
  --mode 1
```

Launcher script:

```bash
./run_visual_servo_combined.sh
```

Example with overrides:

```bash
./run_visual_servo_combined.sh --ip 172.16.0.2 --eMc config/eMc.yaml --mode 2
```

## Main Arguments

- `--eMc`: camera-to-end-effector calibration file
- `--ip`: robot controller IP
- `--tag-size`: physical tag size in meters
- `--desired-factor`: desired distance factor, with distance computed as `tag_size * desired_factor`
- `--adaptive-gain`: enable adaptive visual-servo gain
- `--mode 1`: single-tag mode
- `--mode 2`: sequenced multi-tag mode
- `--tag-z-aligned`: use z-aligned pose estimate
- `--tag-quad-decimate`: AprilTag detector decimation factor
- `--intrinsic`: camera intrinsics XML file
- `--camera-name`: camera name entry inside the XML file
- `--no-convergence-threshold`: disable the convergence stop threshold
- `--verbose`: enable additional logging

With `--tag-size 0.05` and `--desired-factor 8`, the target distance is `0.40 m`.

## GUI Operation

The app window contains:
- a central live camera image
- a top-right `QUIT` button
- a status area below the image
- a bottom control row

Controls:
- `START`: enable robot motion
- `STOP`: disable robot motion
- `HOME`: move to the predefined recovery pose
- `ZOOM OUT`: increase `desired-factor`
- `ZOOM IN`: decrease `desired-factor`
- `QUIT`: close the app

Status area:
- line 1: control hints
- line 2: target distance and current level
- line 3: live operator or safety/event message

Important behavior:
- the app starts with motion disabled
- motion is controlled through `START` / `STOP`, not by clicking the image
- use the in-app `QUIT` button as the primary exit path
- the backend critical-joint monitor remains controller-side and is not currently rendered in the GUI

## Typical Operating Flow

1. Verify camera calibration and hand-eye calibration on your setup.
2. Start the local robot-state API first if you want `arm_moving` forwarded into your FR3/LSL tooling.
3. Launch the app with the correct `--eMc`, robot IP, and tag size.
4. Confirm the AprilTag is visible and the overlay is stable.
5. Adjust the desired distance with `ZOOM OUT` / `ZOOM IN` if needed.
6. Press `START` when you are ready to allow motion.
7. Use `STOP`, `HOME`, or `QUIT` as needed.

## Lost-Target Recovery

If the AprilTag is lost:
- the app first backs off for a short window
- then it applies a biased angular search based on the last observed image drift
- if the tag reappears, normal IBVS resumes

Key tuning parameters live in [src/VisualServoController.cpp](/home/parc/FR3_visual_servo_examples/src/VisualServoController.cpp):
- `scan_backoff_secs_`
- `backoff_speed_`
- `bias_window_secs_`
- `k_ang_`
- `max_angular_`

## Safety Features

Implemented controller-side safeguards:
- servo translational and rotational speed caps
- orientation guard at `+/-45 deg`
- low-pass smoothing of commanded velocity
- soft joint-limit margin guard
- base-frame workspace guard
- minimum camera-to-tag distance stop
- Franka contact, collision, external-wrench, and control-error stop
- manual recovery pose
- explicit motion enable through the GUI

Current workspace bounds:
- `x = 0.20 m` to `0.65 m`
- `y = -0.55 m` to `0.55 m`
- `z = 0.10 m` to `0.85 m`

Current external wrench thresholds:
- `20 N` force magnitude
- `6 Nm` torque magnitude

Important limitation:
- these are controller-side guards, not a certified safety system
- the app still depends on correct calibration, correct workspace tuning, and robot-side protections in libfranka / FR3

Tune the main safety values in [src/VisualServoController.h](/home/parc/FR3_visual_servo_examples/src/VisualServoController.h).

## Robot-State / LSL Integration

The controller posts:
- `{"arm_moving": 1}` when visual servo is actively commanding nontrivial motion
- `{"arm_moving": 0}` when commanded motion drops idle
- a final `arm_moving = 0` on shutdown

Endpoint:

```text
http://127.0.0.1:8765/state
```

If that local state API is not running, the visual servo app still runs, but those state updates will not be consumed downstream.

## Project Structure

```text
FR3_visual_servo_examples/
├── src/
│   ├── main_qt.cpp
│   ├── MainWindow.cpp
│   ├── MainWindow.h
│   ├── VisualServoController.cpp
│   ├── VisualServoController.h
│   └── servoFrankaIBVS_combined.cpp
├── run_visual_servo_combined.sh
├── CMakeLists.txt
└── README.md
```

Notes on source layout:
- `main_qt.cpp` handles CLI parsing and application startup
- `MainWindow.*` owns the Qt GUI
- `VisualServoController.*` owns camera, detection, servo, safety, and robot commands
- `servoFrankaIBVS_combined.cpp` is the older single-file implementation kept as reference; the current binary is built from the Qt path

## Notes

- Default AprilTag family is `36h11`.
- The robot must be reachable and able to enter velocity control before use.
- If no tag is detected, the app performs recovery behavior rather than normal servoing.
- The current default desired level is `8`.

## License

This repository is licensed under **GPL-2.0-or-later**.

Reason: the project links against ViSP (Visual Servoing Platform), and this repository includes/adapts ViSP example-derived code with preserved upstream GPL notices. To keep licensing consistent with that linkage and provenance, the repository is distributed under GNU GPL v2 or any later version.

See [LICENSE](LICENSE) for the full license text and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for dependency notices.
