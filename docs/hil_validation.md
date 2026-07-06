# HIL Validation Procedure — Jetson Brain Against Simulated Sensors

> **HISTORICAL — DO NOT RUN AS WRITTEN.** `agv_hil.launch.py` (used throughout
> this document) and `agv_fusion.launch.py` (referenced in the comparison table)
> were deleted in the 2026-04-13 audit — see `deleted_files` in
> [specs/launch_sequence.yaml](../specs/launch_sequence.yaml). The surviving HIL
> entry point is `agv_hil_full.launch.py`
> (`ros2 launch agv_bringup agv_hil_full.launch.py map:=<map_name>`), and the
> current LAN HIL procedure is
> [docs/validation/RUNBOOK_lan_hil.md](validation/RUNBOOK_lan_hil.md). The topic
> contract and pass/fail criteria below are kept as reference only.

This document defines the exact validation steps for running the AGV autonomy
brain on the Jetson using simulated topics from a PC over the network.

## Prerequisites

### Environment Setup (both machines)

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Jetson-specific

```bash
export CYCLONEDDS_URI=$(ros2 pkg prefix agv_bringup)/share/agv_bringup/config/cyclonedds_hil.xml
source ~/ros2_ws/install/setup.bash
```

### Network Check

```bash
ping -c 3 <sim-host-ip>   # PC sim IP — must match cyclonedds_hil.xml
ros2 topic list | grep agv
```

Expected: `/agv/wheel_odom`, `/agv/imu/data`, `/agv/sim_odom`, `/agv/scan`,
`/agv/joint_states`, `/clock` visible.

If not visible: check ROS_DOMAIN_ID matches, firewall allows UDP 7400-7500,
CycloneDDS XML has correct PC IP.

---

## External Topic Contract

The PC simulator must publish:

| Topic | Type | Rate | frame_id | child_frame_id |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | sim rate | — | — |
| `/agv/wheel_odom` | nav_msgs/Odometry | 50 Hz | `odom` | `base_link` |
| `/agv/joint_states` | sensor_msgs/JointState | 50 Hz | — | — |
| `/agv/imu/data` | sensor_msgs/Imu | 100+ Hz | `imu_link` | — |
| `/agv/sim_odom` | nav_msgs/Odometry | 10 Hz | `map` | `base_link` |
| `/agv/scan` | sensor_msgs/LaserScan | 10 Hz | `laser_frame` | — |

PC must also subscribe to `/agv/cmd_vel` (geometry_msgs/Twist) to close the loop.

---

## Validation A: Dual EKF Fusion

### A1. Launch

```bash
ros2 launch agv_bringup agv_hil.launch.py enable_nav2:=false enable_teleop:=false
```

### A2. Verify topic ingestion

```bash
ros2 topic hz /agv/wheel_odom          # expect ~50 Hz
ros2 topic hz /agv/imu/data            # expect ~100+ Hz
ros2 topic hz /agv/sim_odom            # expect ~10 Hz
ros2 topic hz /agv/odometry/local      # expect ~50 Hz (Jetson output)
ros2 topic hz /agv/odometry/global     # expect ~10 Hz (Jetson output)
```

### A3. Verify TF ownership

```bash
ros2 run tf2_ros tf2_monitor
```

- `odom → base_link`: published by ekf_local at ~50 Hz
- `map → odom`: published by ekf_global at ~10 Hz
- Zero `TF_REPEATED_DATA` warnings
- Zero duplicate publishers

### A4. Verify frame_ids

```bash
ros2 topic echo /agv/odometry/local --once | grep frame_id
# header.frame_id: "odom", child_frame_id: "base_link"

ros2 topic echo /agv/odometry/global --once | grep frame_id
# header.frame_id: "map", child_frame_id: "base_link"
```

### A5. Verify full TF chain

```bash
ros2 run tf2_ros tf2_echo map base_link
# Must show valid transform, updating at ~10 Hz
```

### A6. Record bag

```bash
ros2 bag record /agv/wheel_odom /agv/imu/data /agv/sim_odom \
  /agv/odometry/local /agv/odometry/global /tf /tf_static \
  -o hil_ekf_validation
```

### A7. Pass/fail checklist

| Check | Pass condition | Result |
|---|---|---|
| wheel_odom ingestion | ~50 Hz, no gaps | |
| imu ingestion | ~100 Hz, no gaps | |
| sim_odom ingestion | ~10 Hz | |
| odometry/local output | ~50 Hz | |
| odometry/global output | ~10 Hz | |
| odom→base_link TF | ekf_local, ~50 Hz, no duplicates | |
| map→odom TF | ekf_global, ~10 Hz, no duplicates | |
| tf2_echo map base_link | valid, updating | |
| No TF warnings | zero TF_REPEATED_DATA | |

---

## Validation B: Nav2 Constrained Route

### B1. Prerequisites

- Validation A passes cleanly
- Map YAML exists (from PC sim)
- PC subscribes to `/agv/cmd_vel`

### B2. Launch

```bash
ros2 launch agv_bringup agv_hil.launch.py \
  enable_nav2:=true \
  map:=$HOME/maps/sim_greenhouse.yaml
```

### B3. Verify Nav2 lifecycle

```bash
ros2 lifecycle nodes
# All Nav2 nodes in "active" state

ros2 action list
# /agv/navigate_to_pose must be present
```

### B4. Constrained A→B route

**Start**: robot at sim spawn origin (0, 0, facing +X)
**Goal**: 3 meters forward

```bash
ros2 action send_goal /agv/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### B5. Repeat 5 times

Reset robot to origin between runs. Fill in results:

| Run | Reached goal? | Final error (m) | Time (s) | TF errors? | Notes |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |

### B6. Pass criteria

- 5/5 reach goal within 0.15 m
- Zero Nav2 lifecycle crashes
- Zero TF conflicts
- cmd_vel being published by controller
- No stuck state (progress checker timeout = 10 s)

### B7. Common failure modes

| Symptom | Likely cause | Fix |
|---|---|---|
| Nav2 nodes don't start | lifecycle_manager timeout | check map loaded, increase timeout |
| Planner fails "no valid path" | goal in obstacle or unknown space | verify map, try different goal |
| Controller oscillates | RegulatedPurePursuit tuning | increase lookahead_dist |
| Robot doesn't move in sim | PC not subscribed to cmd_vel | check PC sim diff_drive plugin |
| TF timeout in costmap | use_sim_time mismatch | verify all nodes have use_sim_time=true |
| Goal never reached | tolerance too tight | increase xy_goal_tolerance to 0.25 |

---

## Validation C: Marker Pipeline Foundations

`agv_markers` is `post_mvp_stretch` — no code exists yet.

### C1. Interface contract (from specs/interfaces.yaml)

- Input: `/{robot_namespace}/tag_detections` (AprilTagDetectionArray)
- Output: `/{robot_namespace}/marker_pose` (PoseWithCovarianceStamped)
- Output: `/{robot_namespace}/marker_detected` (String)

### C2. EKF integration path

When markers are implemented, `marker_pose` is added as `odom2` in
`ekf_global.yaml` / `ekf_global_hil.yaml`. Config change only — no code.

### C3. Status

No blocker. Architecture supports future marker integration.

---

## HIL Mode vs Real Operation

| Aspect | HIL Mode | Real Mode |
|---|---|---|
| Wheel odom | PC sim `/agv/wheel_odom` | agv_odrive CAN node |
| IMU | PC sim `/agv/imu/data` | ZED 2i `/zed/zed_node/imu/data` |
| Visual odom | PC sim `/agv/sim_odom` | cuVSLAM `/visual_slam/tracking/odometry` |
| Lidar | PC sim `/agv/scan` | real lidar or nvblox depth-to-scan |
| Time | `/clock` from PC | system clock |
| Motors | cmd_vel → PC sim | cmd_vel → agv_odrive → CAN |
| Launch | `agv_hil.launch.py` | `agv_fusion.launch.py` + nav |
| EKF configs | `ekf_*_hil.yaml` | `ekf_local.yaml` / `ekf_global.yaml` |

---

## Blockers Before Real Sensors

1. Global EKF validation with real cuVSLAM (currently only local EKF validated)
2. Real occupancy grid map (via `agv_mapping.launch.py` in greenhouse)
3. Lidar source decision (real sensor or nvblox depth-to-LaserScan)
4. Real Nav2 bringup launch (using real EKF configs, not HIL)
5. Right-wheel asymmetry fix (`right_scale` tuning or mechanical)
