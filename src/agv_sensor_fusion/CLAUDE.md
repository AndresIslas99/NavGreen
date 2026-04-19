# agv_sensor_fusion

Dual Extended Kalman Filter sensor fusion using `robot_localization`. Local EKF (50 Hz) fuses
wheel odometry + filtered IMU for continuous odom->base_link. Global EKF (10 Hz) fuses local
estimate + cuVSLAM + optional AprilTag markers for map->odom. IMU vibration filter removes
greenhouse floor noise before EKF ingestion. Includes fusion health monitoring.

## Nodes

- **imu_filter_node** (C++17): Butterworth 2nd-order low-pass on gyro (10 Hz cutoff) and accel (5 Hz cutoff). Removes mechanical vibrations from greenhouse floor. Orientation passes through unfiltered. **Must launch before EKF** (t=3.5s in agv_full).
- **ekf_local** (robot_localization/ekf_node): Wheel odom + filtered IMU -> odom->base_link TF (50 Hz)
- **ekf_global** (robot_localization/ekf_node): Local + cuVSLAM + markers -> map->odom TF (10 Hz)
- **fusion_monitor_node** (C++17): Per-sensor health tracking, covariance diagnostics, pose republishing
- **covariance_override_node** (C++17): Fills zero covariances from Gazebo sim (simulation only)

## IMU Pipeline

```
ZED BMI088 @ 200Hz → /agv/zed/imu/data (raw, vibration noise)
  → imu_filter_node (Butterworth 2nd order)
    → gyro: 10Hz cutoff (removes >10Hz vibrations)
    → accel: 5Hz cutoff (removes >5Hz vibrations)
    → orientation: pass-through (no lag)
  → /agv/imu/filtered (clean)
    → ekf_local (50Hz fusion)
```

## Topics

**Published (imu_filter):**
- `imu/filtered` (sensor_msgs/Imu, 200 Hz) — Vibration-filtered IMU for EKF

**Published (fusion_monitor):**
- `pose` (PoseWithCovarianceStamped, 10 Hz) — Global EKF pose republished per spec
- `/diagnostics` (DiagnosticArray, 1 Hz) — Localization health with per-sensor status
- `/agv/cuvslam_tracking_ok` (std_msgs/Bool) — cuVSLAM tracking boolean for downstream

**Published (EKF nodes):**
- `odometry/local` (Odometry, 50 Hz) — Local filtered estimate
- `odometry/global` (Odometry, 10 Hz) — Global filtered estimate
- TF: `odom->base_link` (local), `map->odom` (global)

**Subscribed (imu_filter):**
- `imu/raw` (remapped to `/agv/zed/imu/data`) — Raw ZED IMU at 200 Hz

**Subscribed (ekf_local):**
- `wheel_odom` — Odometry from odrive_can_node (50 Hz, with dynamic covariance)
- `/agv/imu/filtered` — Filtered IMU (was `/agv/zed/imu/data` before imu_filter)

**Subscribed (fusion_monitor):**
- `odometry/global`, `odometry/local` — EKF outputs for health check
- `/agv/wheel_odom`, `/visual_slam/tracking/odometry`, `/agv/imu/filtered`, `/agv/marker_pose` — Per-sensor health tracking
- `/visual_slam/status` — cuVSLAM tracking state string

## Parameters

**imu_filter_node:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `gyro_cutoff_hz` | `10.0` | Angular velocity low-pass cutoff (Hz) |
| `accel_cutoff_hz` | `5.0` | Linear acceleration low-pass cutoff (Hz) |
| `sample_rate` | `200.0` | IMU sample rate (must match ZED sensors_pub_rate) |

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

- `config/imu_filter.yaml` — Butterworth filter: gyro 10Hz, accel 5Hz cutoff at 200Hz sample rate
- `config/ekf_local.yaml` — Local EKF: wheel_odom (odom0) + filtered IMU (imu0), 2D mode, dynamic process noise. IMU yaw covariance: 0.02 rad² (trusts gyro over encoder). IMU vyaw covariance: 0.0005.
- `config/ekf_global.yaml` — Global EKF: local odom (odom0, differential) + cuVSLAM (odom1, differential) + markers (pose0, absolute). cuVSLAM rejection: pose=3.5, twist=2.5.
- `launch/fusion.launch.py` — Launches ekf_local, ekf_global, fusion_monitor (imu_filter launched separately in agv_bringup)

## Key Design Decisions

- **IMU filtered before EKF**: Greenhouse floor vibrations at >10Hz corrupt gyro readings. Butterworth 2nd-order gives -40dB/decade roll-off with zero phase distortion at DC.
- **IMU yaw trusted over encoders**: IMU yaw covariance (0.02 rad²) tighter than wheel odom yaw (0.03 base). During rotation, encoder covariance inflates 5x per rad/s, making IMU dominant for heading.
- Local EKF uses **absolute** wheel odometry; Global EKF uses **differential** mode to prevent double-counting drift
- cuVSLAM rejection thresholds: pose=3.5, twist=2.5 (Mahalanobis distance)
- AprilTag rejection threshold: 3.0
- Dynamic process noise scales Q with velocity (realistic uncertainty at speed)
- **TF ownership**: Only ekf_local publishes odom→base_link, only ekf_global publishes map→odom. cuVSLAM and SLAM Toolbox TF publishing are disabled via YAML overrides.

## Dependencies

- robot_localization, rclcpp, nav_msgs, sensor_msgs, diagnostic_msgs, tf2

## Improvement Opportunities

- Add unit tests for fusion_monitor and imu_filter_node
- Add sensor latency monitor (detect clock sync issues between sources)
- Add graceful degradation when EKF frequency drops below target
- Consider adaptive filter cutoff based on robot velocity (higher speed = higher cutoff)
