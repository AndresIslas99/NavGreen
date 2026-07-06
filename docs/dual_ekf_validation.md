# Dual EKF Validation Procedure

> **HISTORICAL — DO NOT RUN AS WRITTEN.** Four of the five launch files this
> procedure scripts against (`agv_robot_core.launch.py`, `agv_teleop.launch.py`,
> `agv_ekf_local_test.launch.py`, `agv_fusion.launch.py`) were deleted in the
> 2026-04-13 audit — see `deleted_files` in
> [specs/launch_sequence.yaml](../specs/launch_sequence.yaml). The surviving
> entry points are `agv_full.launch.py` (production; runs ekf_local + ekf_global),
> `agv_mapping.launch.py` (commissioning / map creation), and
> `agv_hil_full.launch.py` (HIL). The dual-EKF TF ownership rule remains
> canonical (ekf_local owns `odom → base_link`, ekf_global owns `map → odom`),
> but the step-by-step commands below must be re-derived against
> `agv_full.launch.py` before use. For the current HIL validation loop see
> [docs/validation/RUNBOOK_lan_hil.md](validation/RUNBOOK_lan_hil.md).

## TF Ownership Authority Table

Historical table — launch modes as they existed before the 2026-04-13 audit.
The authoritative launch inventory is `specs/launch_sequence.yaml`.

| Launch mode | `odom → base_link` | `map → odom` | cuVSLAM TF | Notes |
|---|---|---|---|---|
| `agv_robot_core.launch.py` | — | — | not running | Motor control only |
| `agv_teleop.launch.py` | — | — | not running | Teleop only, no localization |
| `agv_ekf_local_test.launch.py` | **ekf_local** | — | not running | Validation step 1-2 |
| `agv_mapping.launch.py` | **cuVSLAM** | **cuVSLAM** | **enabled** | Commissioning / map creation |
| `agv_fusion.launch.py` | **ekf_local** | **ekf_global** | **disabled** | Production fusion mode |

**Rule**: In any given launch mode, exactly ONE node publishes each transform. Duplicate publishers cause `TF_REPEATED_DATA` warnings and unstable localization.

---

## Validation Steps

### Prerequisites

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select agv_bringup agv_slam agv_sensor_fusion
source install/setup.bash
```

### Step 1: Local EKF — wheel odom only (no camera)

**Goal**: Verify `odom→base_link` from local EKF using wheel odometry alone.

```bash
# Terminal 1: Launch
ros2 launch agv_bringup agv_ekf_local_test.launch.py

# Terminal 2: Record bag from the start
ros2 bag record /agv/wheel_odom /agv/odometry/local /agv/drive_debug /tf /tf_static \
  -o ~/bags/ekf_step1_odom_only

# Terminal 3: Verify frame_ids
ros2 topic echo /agv/wheel_odom --once
#   MUST show: header.frame_id == "odom"
#   MUST show: child_frame_id == "base_link"

# Terminal 4: Verify rates
ros2 topic hz /agv/wheel_odom        # expect ~50 Hz
ros2 topic hz /agv/odometry/local    # expect ~50 Hz

# Terminal 5: Verify TF
ros2 run tf2_ros tf2_monitor
#   MUST show: odom → base_link (from ekf_local)
#   MUST NOT show: map → odom (no global filter)
```

**Pass criteria**:
- [ ] `wheel_odom` publishing at ~50 Hz
- [ ] `odometry/local` publishing at ~50 Hz
- [ ] `wheel_odom.header.frame_id == "odom"`
- [ ] `wheel_odom.child_frame_id == "base_link"`
- [ ] `odom→base_link` TF present and updating
- [ ] No TF warnings about duplicate publishers
- [ ] Drive robot forward → pose.x increases
- [ ] Release joystick → pose stable, no drift
- [ ] Rosbag captured

### Step 2: Local EKF — wheel odom + IMU

**Goal**: Verify IMU fusion improves orientation estimate.

```bash
# Terminal 1: Launch with ZED
ros2 launch agv_bringup agv_ekf_local_test.launch.py enable_zed:=true

# Terminal 2: Record bag
ros2 bag record /agv/wheel_odom /agv/odometry/local /zed/zed_node/imu/data /tf /tf_static \
  -o ~/bags/ekf_step2_with_imu

# Terminal 3: Verify IMU frame_id
ros2 topic echo /zed/zed_node/imu/data --once
#   Note the header.frame_id (e.g. "zed_imu_link" or "zed2i_imu_link")
#   EKF must be able to resolve this frame via TF

# Terminal 4: Verify rates
ros2 topic hz /zed/zed_node/imu/data   # expect ~200-400 Hz
ros2 topic hz /agv/odometry/local      # expect ~50 Hz
```

**Pass criteria**:
- [ ] IMU data flowing at ~200+ Hz
- [ ] No EKF warnings about frame lookup failures
- [ ] No NaN values in `odometry/local`
- [ ] Orientation smoother than step 1 (roll/pitch from IMU)
- [ ] Rosbag captured

### Step 3: Full dual EKF with cuVSLAM

**Goal**: Verify complete TF chain `map → odom → base_link` with no conflicts.

```bash
# Terminal 1: Launch full fusion
ros2 launch agv_bringup agv_fusion.launch.py

# Terminal 2: Record bag
ros2 bag record /agv/wheel_odom /agv/odometry/local /agv/odometry/global \
  /visual_slam/tracking/odometry /agv/drive_debug /tf /tf_static \
  -o ~/bags/ekf_step3_full_fusion

# Terminal 3: Verify cuVSLAM frame_ids
ros2 topic echo /visual_slam/tracking/odometry --once
#   Note header.frame_id and child_frame_id
#   Must match what ekf_global.yaml expects

# Terminal 4: Verify rates
ros2 topic hz /agv/odometry/local                  # expect ~50 Hz
ros2 topic hz /agv/odometry/global                 # expect ~10 Hz
ros2 topic hz /visual_slam/tracking/odometry       # expect ~10-15 Hz

# Terminal 5: Verify TF tree
ros2 run tf2_ros tf2_monitor
#   MUST show: odom → base_link (ekf_local, ~50 Hz)
#   MUST show: map → odom (ekf_global, ~10 Hz)
#   MUST show: base_link → zed_camera_link (static)
#   MUST NOT show: TF_REPEATED_DATA or duplicate warnings
```

**Pass criteria**:
- [ ] Both EKF outputs at expected rates
- [ ] TF tree `map → odom → base_link → zed_camera_link` complete within 5s
- [ ] Zero `TF_REPEATED_DATA` or duplicate transform warnings
- [ ] Short trayecto (forward + turn) with stable, reasonable pose
- [ ] Release joystick → odom stable, no residual drift
- [ ] Rosbag captured

### Step 4: Safety behavior under fusion

**Only after Step 3 passes cleanly.**

```bash
# With agv_fusion.launch.py running:
# 1. Release joystick → true zero, no creep
# 2. E-stop from UI → motors stop immediately
```

**Pass criteria**:
- [ ] Release = true zero (no hum, no creep)
- [ ] E-stop = immediate stop

---

## Degradation Tests (Phase C — optional, after baseline is clean)

Only attempt after all Step 1-4 pass:

1. **Kill cuVSLAM**: `ros2 lifecycle set /visual_slam shutdown` → verify local EKF continues, global holds last correction
2. **Restart cuVSLAM**: re-launch → verify global EKF resumes correction
3. **Cover camera lens**: verify local EKF sustains odometry from wheel encoders alone

---

## Gate: Do NOT proceed to Nav2 until

- [ ] `odom → base_link` clean from local EKF
- [ ] `map → odom` clean from global EKF
- [ ] Zero TF duplicate warnings
- [ ] Short real trayecto with stable pose
- [ ] Rosbag captured for each step
