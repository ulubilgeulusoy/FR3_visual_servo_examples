# FR3 Visual Servo Examples

Visual servoing application for the Franka Research 3 (FR3) using ViSP, `libfranka`, and an Intel RealSense camera.

This codebase corresponds to the `visp_gui_combined` version of the project. It builds a single combined application, `servoFrankaIBVS_combined`, from [`src/servoFrankaIBVS_combined.cpp`](/home/parc/FR3_visual_servo_examples/src/servoFrankaIBVS_combined.cpp). It extends ViSP's `servoFrankaIBVS` example with:

- adjustable AprilTag size and desired standoff distance
- two operating modes selected with `--mode`
- lost-target recovery when the tag leaves view
- controller-side safety guards
- local `arm_moving` state updates for FR3 control / LSL tooling

## What It Does

The application detects an AprilTag in the RealSense image, estimates its pose, and commands Cartesian camera-frame velocity so the robot approaches a desired viewing pose.

Current modes:
- `--mode 1`: single-tag visual servoing
- `--mode 2`: sequenced multi-tag behavior that cycles target IDs `{1, 2, 1, ...}`

During operation, the app provides on-screen controls for:
- `START` / `STOP` motion enable
- `HOME` recovery pose
- `ZOOM OUT` / `ZOOM IN` desired-distance adjustment
- `QUIT` application exit

Keyboard shortcuts:
- `+` / `-`: adjust `desired-factor`
- `r`: move to the recovery pose

## Requirements

- FR3 reachable on the network and ready for velocity control
- `libfranka`
- ViSP built with Franka and RealSense support
- Intel RealSense SDK 2.x
- CMake >= 3.10
- C++17-capable compiler
- camera-to-end-effector calibration file such as `config/eMc.yaml`
- printed AprilTag with known physical size

If this is a new robot/camera setup, first complete calibration and verify the upstream beginner example works on your hardware:

https://github.com/yiherngang/Franka-Research-3-with-ROS-2-Imvia-lab/tree/main?tab=readme-ov-file#70-visual-servoing-with-franka-research-3

## Build

Build from the repo root:

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DViSP_DIR=~/visp_install/lib/cmake/visp
make -j$(nproc)
```

This version builds:
- `servoFrankaIBVS_combined`

## Run

From the repo root:

```bash
./build/servoFrankaIBVS_combined \
  --eMc config/eMc.yaml \
  --ip 172.16.0.2 \
  --tag-size 0.05 \
  --desired-factor 8 \
  --adaptive-gain \
  --mode 1
```

You can also use:

```bash
./run_visual_servo_combined.sh
```

Main arguments:
- `--eMc`: camera-to-end-effector calibration file
- `--ip`: FR3 IP address
- `--tag-size`: physical AprilTag size in meters
- `--desired-factor`: desired distance factor, where distance is `tag_size * desired_factor`
- `--adaptive-gain`: enable adaptive visual-servo gain
- `--mode 1`: single-tag mode
- `--mode 2`: sequenced multi-tag mode

With `--tag-size 0.05` and `--desired-factor 8`, the target distance is `0.40 m`.

## Typical Use Flow

1. Verify camera calibration and hand-eye calibration.
2. Build the project and confirm `config/eMc.yaml` or your own calibration file is available.
3. Place the AprilTag fully in view and confirm lighting/detection quality.
4. Start the optional local robot-state API first if you want `arm_moving` forwarded into the FR3 LSL pipeline.
5. Launch `servoFrankaIBVS_combined` with the correct robot IP and calibration file.
6. Confirm the tag overlay is stable before enabling motion.
7. Use `START`, `STOP`, `HOME`, and desired-distance controls as needed while monitoring the robot closely.

## Lost-Target Recovery

If the AprilTag is lost, the controller:

1. backs off briefly to widen the field of view
2. applies a biased angular search using the last observed image drift
3. resumes normal IBVS immediately when the tag reappears

Main tuning parameters in [`src/servoFrankaIBVS_combined.cpp`](/home/parc/FR3_visual_servo_examples/src/servoFrankaIBVS_combined.cpp):
- `k_ang`
- `bias_boost`
- `max_angular`
- `bias_window_secs`
- `scan_backoff_secs`

## Safety Features

Implemented controller-side safeguards:

- translational and rotational speed caps
- orientation guard
- low-pass smoothing of commanded velocity
- soft joint-limit guard
- workspace guard in the robot base frame
- minimum camera-to-tag distance stop
- contact / collision / control-error / external-wrench stop
- manual recovery pose
- explicit motion enable through the footer controls

These values are tuned in [`src/servoFrankaIBVS_combined.cpp`](/home/parc/FR3_visual_servo_examples/src/servoFrankaIBVS_combined.cpp) and should be reviewed for your setup.

Important limitation:
- these are controller-side guards, not a certified safety system

## Robot-State / LSL Integration

The controller posts `arm_moving` updates to:

```text
http://127.0.0.1:8765/state
```

Behavior:
- posts `{"arm_moving": 1}` while commanding nontrivial motion
- posts `{"arm_moving": 0}` when motion returns idle
- forces a final `arm_moving = 0` on clean exit

If the local state API is not running, the visual servo app still runs, but those state updates will not be consumed downstream.

## Project Structure

```text
FR3_visual_servo_examples/
├── src/
│   └── servoFrankaIBVS_combined.cpp
├── run_visual_servo_combined.sh
├── CMakeLists.txt
└── README.md
```

## Notes

- Default AprilTag family is `36h11`.
- The robot must be in velocity control mode before use.`r`n- If no tag is detected, the app performs recovery behavior instead of normal servoing.`r`n- Visual-servo `arm_moving` state is derived from commanded ViSP camera-frame velocity, not external ROS joint-state inference.`r`n`r`n## License`r`n`r`nThis repository is licensed under **GPL-2.0-or-later**.`r`n`r`nReason: the project links against ViSP (Visual Servoing Platform), and this repository includes/adapts ViSP example-derived code with preserved upstream GPL notices. To keep licensing consistent with that linkage and provenance, the repository is distributed under GNU GPL v2 or any later version.`r`n`r`nSee [LICENSE](LICENSE) for the full license text and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for dependency notices.`r`n

