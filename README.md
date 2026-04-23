# FR3 Visual Servo Examples

Visual servoing application for the Franka Research 3 (FR3) robot using ViSP and an Intel RealSense camera.  
This project extends ViSP’s `servoFrankaIBVS` example with configurable tag size, adjustable desired distance, and lost-target recovery behavior when the AprilTag is not visible.

## User Guide

This application moves the FR3 by visual servoing on an AprilTag seen by the RealSense camera. In normal operation, the camera detects the tag, estimates its pose, and commands Cartesian velocity so the robot approaches the desired viewing position while respecting the configured safety limits.

Typical use flow:
1. Complete camera calibration and verify the upstream beginner example works on your setup.
2. Build this project and confirm you have a valid hand-eye calibration file such as `config/eMc.yaml`.
3. Place a supported AprilTag in the camera view with a known physical size.
4. Put the robot in the correct mode for velocity control and make sure the FR3 is reachable at the IP you will pass on the command line.
5. Start the optional local robot-state API first if you want `arm_moving` updates forwarded to the LSL pipeline.
6. Launch `servoFrankaIBVS_combined` with the calibration file, robot IP, tag size, and desired distance settings.
7. Watch the camera window and verify the tag is detected before allowing the robot to continue moving toward the target pose.

What you need before running:
- A working FR3 + `libfranka` setup.
- ViSP built with Franka and RealSense support.
- An Intel RealSense camera mounted consistently with your calibration.
- A printed AprilTag of known size.
- The camera-to-end-effector calibration file passed through `--eMc`.

How to run:
- Build the project from the repo root:
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DViSP_DIR=~/visp_install/lib/cmake/visp
make -j$(nproc)
```
- Start the application from the repo root with your robot IP:
```bash
./build/servoFrankaIBVS_combined \
  --eMc config/eMc.yaml \
  --ip 172.16.0.2 \
  --tag-size 0.05 \
  --desired-factor 7 \
  --adaptive-gain \
  --mode 1
```

What the main arguments do:
- `--eMc`: path to the hand-eye calibration file.
- `--ip`: FR3 IP address.
- `--tag-size`: physical AprilTag size in meters.
- `--desired-factor`: desired standoff distance, computed as `tag_size * desired_factor`.
- `--adaptive-gain`: enables adaptive visual-servo gain for smoother convergence.
- `--mode 1`: single-tag behavior.
- `--mode 2`: sequenced multi-tag behavior.

How to use the app while it is running:
- Keep the target AprilTag visible in the camera window.
- Use `+` and `-` on the keyboard to increase or decrease `desired-factor` live.
- On a touchscreen or with a mouse, press the on-screen `+` and `-` buttons in the lower-right corner.
- Watch for lost-target behavior: if the tag disappears, the robot briefly backs off and biases its search using the last observed image motion.
- Stop the run immediately if detection is unstable, calibration looks wrong, or the robot motion does not match the camera image.

Recommended operating practice:
- Start with a conservative `desired-factor` and low-risk robot pose.
- Make sure the tag is well lit and fully visible before engaging.
- Keep people and obstacles clear of the robot path.
- Verify tag size, calibration, and camera mounting whenever servo behavior looks inconsistent.

## Camera Calibration and Beginner Example

Before doing anything with this visual servoing example, follow the instructions from this GitHub Repo: https://github.com/yiherngang/Franka-Research-3-with-ROS-2-Imvia-lab/tree/main?tab=readme-ov-file#70-visual-servoing-with-franka-research-3 and make sure you perform the camera calibration correctly, and the provided visual servoing example works with your robot arm.

## Lost-target recovery

When the AprilTag is lost, the controller uses the
**last known image drift** of the tag to **turn directly toward where it was heading**.

### Logic
1. **Backoff window (default 3.5 s)**  
   Gently move backward to widen FOV.
2. **Biased turn window (default 3.5 s)**  
   Apply an angular velocity bias from the last centroid motion in pixels/sec:  
   - Horizontal drift → yaw/roll bias  
   - Vertical drift → pitch bias  
   The bias decays over time if the tag is still not found.

If the tag reappears, the timer resets and normal IBVS resumes.

### Tuning knobs (in `src/servoFrankaIBVS_combined.cpp`)
- `k_ang` (px/s → rad/s gain) and optional `bias_boost` multiply how hard we turn.
- `max_angular` caps angular speed.
- `bias_window_secs` controls how long we keep the pure biased turn before decaying.
- `scan_backoff_secs` controls the initial backoff duration.
- Optional: a small `v_c[2] = -0.01` m/s during biased turn can help re-acquire.

Example safe settings:
```cpp
double k_ang = vpMath::rad(0.12);
double bias_boost = 1.4;
double max_angular = vpMath::rad(30);
double bias_window_secs = 3.5;
double scan_backoff_secs = 3.5;
```

## Safety features

- **Servo speed caps**: Translational and rotational camera-frame velocities are clamped (`servo_max_linear`, `servo_max_angular`) to keep approach slow.
- **Orientation guard**: If the current-to-desired tag orientation error exceeds ±95°, commanded velocities are zeroed until the error returns below the threshold (`orientation_stop_thresh`).
- **Velocity smoothing**: A first-order low-pass filter (`vel_smooth_alpha`) blends new commands with the previous ones to reduce twitchiness while keeping the caps/guard in place.

Tune these in `src/servoFrankaIBVS_combined.cpp` as needed for your setup.

## Robot-State / LSL Integration

`servoFrankaIBVS_combined.cpp` now publishes `arm_moving` directly to the local robot-state API used by the FR3 control / LSL tooling.

Current behavior:
- when visual servo is actively commanding a nontrivial Cartesian velocity, it posts `{"arm_moving": 1}` to `http://127.0.0.1:8765/state`
- when commanded velocity drops to idle, it posts `{"arm_moving": 0}`
- on clean app exit, it forces a final `arm_moving = 0` update

This is intentionally controller-side rather than ROS-topic-side, since the ViSP/libfranka control path is the most reliable place to determine whether the robot is actually being commanded to move during visual servoing.

Prerequisite:
- the local state API (`robot_state_api.py` from the FR3 control GUI repo) must be running if you want these updates to appear in the `FR3_State` LSL stream


## 📦 Dependencies

- [libfranka](https://frankaemika.github.io/docs/installation_linux.html) (for FR3)
- [ViSP](https://visp.inria.fr) >= 3.6, built with RealSense and Franka support
- [Intel RealSense SDK 2.x](https://github.com/IntelRealSense/librealsense)
- CMake >= 3.10
- A C++17 compiler (GCC >= 9 recommended)

---

## 🔧 Building ViSP (once)

```bash
# Clone ViSP
git clone https://github.com/lagadic/visp.git
cd visp
mkdir build && cd build

# Configure + build + install
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/visp_install -DBUILD_EXAMPLES=OFF -DBUILD_DEMOS=OFF
make -j$(nproc)
make install
```

This will install ViSP into `~/visp_install`.

---

## 🚀 Building (CHRPS + Combined)

```bash
git clone https://github.com/ulubilgeulusoy/FR3_visual_servo_examples.git
cd FR3_visual_servo_examples
mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DViSP_DIR=~/visp_install/lib/cmake/visp
make -j$(nproc)

# Or build just the combined binary
make servoFrankaIBVS_combined
```

---

## ▶️ Running The Combined App

Binary: `servoFrankaIBVS_combined` (also wrapped by `run_visual_servo_combined.sh`)

If you want `arm_moving` to propagate into the FR3 LSL stream during visual servoing, make sure the local state API is already running on the same machine at `http://127.0.0.1:8765/state`.

Example run (FR3 connected at `172.16.0.2`):

```bash
./build/servoFrankaIBVS_combined \
  --eMc config/eMc.yaml \
  --ip 172.16.0.2 \
  --tag-size 0.05 \
  --desired-factor 7 \
  --adaptive-gain \
  --mode 1
```

- `--tag-size` sets the physical AprilTag size in meters (default `0.05 m`)
- `--desired-factor` sets the desired camera distance as `tag_size * desired_factor`
  With `tag-size=0.05`, `desired-factor=7` means `0.35 m` or `35 cm`
  Safety floor: the minimum allowed `desired-factor` is `3`
- `--eMc` provides the camera-to-end-effector calibration file
- `--adaptive-gain` improves convergence
- `--mode` selects single-tag (`1`) or sequenced multi-tag (`2`) behavior

While the camera window is open, you can adjust `desired-factor` live:
- Keyboard: `+` / `-`
- Touchscreen or mouse: tap the on-screen `+` and `-` buttons in the bottom-right corner

The app now opens only the camera view window; the old graph popup has been removed.

---

## ▶️ Combined App (mode switch)

Modes:
- `--mode 1` (default): single-tag CHRPS behavior — expects exactly one detected tag; keeps CHRPS safety features (velocity caps, orientation guard, low-pass smoothing) and rolling 2 s corner trajectories.
- `--mode 2`: multi-tag sequence like the Investment variant — cycles AprilTag IDs `{1,2,1,...}` every 5 s; only servos to the current target ID; displays the active target and lost-target overlays. Safety caps and smoothing remain enabled.

Example:
```bash
# mode 1 (single tag, default)
./servoFrankaIBVS_combined --eMc config/eMc.yaml --ip 172.16.0.2 --mode 1

# mode 2 (tag ID sequence)
./servoFrankaIBVS_combined --eMc config/eMc.yaml --ip 172.16.0.2 --mode 2
```

Helper script (builds then runs combined):
```bash
MODE=2 ./run_visual_servo_combined.sh   # set MODE=1 or 2
```

## 📂 Project Structure

```
FR3_visual_servo_examples/
├── src/
│   └── servoFrankaIBVS_combined.cpp
├── run_visual_servo_combined.sh
├── CMakeLists.txt
└── README.md
```

---

## 📝 Notes

- Default tag family is `36h11`.
- The robot must be in **velocity control mode** and connected before running.
- If no tag is detected, the robot moves back briefly to try to reacquire it.
- Visual-servo `arm_moving` state is derived from the commanded ViSP camera-frame velocity, not from external ROS joint-state inference.

---
