# agv_sensor_fusion

Dual Extended Kalman Filter sensor fusion using `robot_localization`. Local EKF (50 Hz) fuses
wheel odometry + IMU for continuous odom->base_link. Global EKF (10 Hz) fuses local estimate +
cuVSLAM + optional AprilTag markers for map->odom. Includes fusion health monitoring.

## Nodes

- **ekf_local** (robot_localization/ekf_node): Wheel odom + IMU -> odom->base_link TF (50 Hz)
- **ekf_global** (robot_localization/ekf_node): Local + cuVSLAM + markers -> map->odom TF (10 Hz)
- **fusion_monitor_node** (C++17): Per-sensor health tracking, covariance diagnostics, pose republishing
- **covariance_override_node** (C++17): Fills zero covariances from Gazebo sim (simulation only)

## Topics

**Published (fusion_monitor):**
- `pose` (PoseWithCovarianceStamped, 10 Hz) — Global EKF pose republished per spec
- `/diagnostics` (DiagnosticArray, 1 Hz) — Localization health with per-sensor status
- `/agv/cuvslam_tracking_ok` (std_msgs/Bool) — cuVSLAM tracking boolean for downstream

**Published (EKF nodes):**
- `odometry/local` (Odometry, 50 Hz) — Local filtered estimate
- `odometry/global` (Odometry, 10 Hz) — Global filtered estimate
- TF: `odom->base_link` (local), `map->odom` (global)

**Subscribed (fusion_monitor):**
- `odometry/global`, `odometry/local` — EKF outputs for health check
- `/agv/wheel_odom`, `/visual_slam/tracking/odometry`, `/zed/zed_node/imu/data`, `/agv/marker_pose` — Per-sensor health tracking
- `/visual_slam/status` — cuVSLAM tracking state string

## Parameters

**fusion_monitor_node:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `pose_rate_hz` | `10.0` | Pose republish rate |
| `covariance_warn_threshold` | `0.5` | Covariance warn level |
| `covariance_error_threshold` | `2.0` | Covariance error level |
| `stale_timeout_s` | `2.0` | EKF staleness threshold |
| `cuvslam_status_topic` | `"/visual_slam/status"` | cuVSLAM status topic |

**covariance_override_node** (9 params): Override values for odom/imu/vslam covariances (simulation only).

## Configuration

- `config/ekf_local.yaml` — Local EKF: wheel_odom (odom0) + IMU (imu0), 2D mode, dynamic process noise
- `config/ekf_global.yaml` — Global EKF: local odom (odom0, differential) + cuVSLAM (odom1, differential) + markers (pose0, absolute)
- `launch/fusion.launch.py` — Launches ekf_local, ekf_global, fusion_monitor

## Key Design Decisions

- Local EKF uses **absolute** wheel odometry; Global EKF uses **differential** mode to prevent double-counting drift
- cuVSLAM rejection thresholds: pose=5.0, twist=3.0 (Mahalanobis distance)
- AprilTag rejection threshold: 3.0
- Dynamic process noise scales Q with velocity (realistic uncertainty at speed)
- Sensor health tracks rate, age, and staleness per source (5-second rolling window)

## Dependencies

- robot_localization, rclcpp, nav_msgs, sensor_msgs, diagnostic_msgs, tf2

## Improvement Opportunities

- Add unit tests for fusion_monitor (currently zero test coverage)
- Add sensor latency monitor (detect clock sync issues between sources)
- Make cuVSLAM status topic auto-discoverable across isaac_ros versions
- Add graceful degradation when EKF frequency drops below target
- Reduce stale_timeout_s from 2.0s for faster error detection in diagnostics
