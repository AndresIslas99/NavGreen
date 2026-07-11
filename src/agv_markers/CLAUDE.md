# agv_markers

AprilTag-based global pose correction and drift mitigation. Detects tag36h11 markers,
estimates robot pose via solvePnP, and publishes corrections to the global EKF.
Triggers relocalization (EKF hard reset) when drift exceeds threshold.
Post-MVP priority — supplemental to visual SLAM + wheel odometry fusion.

## Nodes

- **marker_correction_node** (C++17): Processes AprilTag detections, estimates pose
  from camera observations, publishes corrections or triggers relocalization.

## Topics

**Published:**
- `marker_pose` (PoseWithCovarianceStamped) — Robot pose correction in map frame, covariance scaled by range
- `marker_detected` (String) — `"tag_<id>"` on each detection (throttled 2s)

**Subscribed:**
- `/detections` (AprilTagDetectionArray) — AprilTag detections from isaac_ros_apriltag
- `/zed/zed_node/left/camera_info` (CameraInfo) — Camera intrinsics for solvePnP
- `odometry/global` (Odometry) — Current EKF pose for drift detection

## Service Clients

- `set_pose` (robot_localization/SetPose) — Hard-reset EKF on relocalization events

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `markers_registry_file` | `""` | YAML file with marker ID -> map pose |
| `max_detection_range` | `5.0` | Ignore detections beyond (m) |
| `tag_size` | `0.2` | Global physical tag size (m); a per-tag `size:` field in the registry overrides it (HIGH-04-04) |
| `covariance_xy` | `0.01` | Base XY covariance (m^2) |
| `covariance_yaw` | `0.03` | Base yaw covariance (rad^2) |
| `relocalization_threshold` | `2.0` | Drift threshold for EKF hard reset (m) |
| `min_confidence` | `50.0` | Minimum decision_margin for relocalization |
| `relocalization_cooldown_ms` | `500` | Cooldown after set_pose to let EKF settle |
| `camera_frame` | `"zed_left_camera_frame"` | TF frame the solvePnP tvec is transformed through |
| `camera_frame_is_optical` | `false` | Set true if `camera_frame` is an optical-convention frame (x right, y down, z forward); when false the fixed optical->body rotation is applied to tvec before the TF |

## Key Algorithms

- **Pose estimation**: cv::solvePnP from 4 tag corners + camera intrinsics
- **Heading extraction**: Rodrigues rotation matrix -> atan2(R[0,0], R[2,0]) for independent heading correction
- **Covariance scaling**: Quadratic with range: `cov = base * (1 + (range/2)^2)`
- **Drift detection**: Euclidean distance between estimated and EKF pose
- **Relocalization**: Async set_pose call + cooldown to prevent EKF correction race condition

## Configuration

- `config/markers_registry.yaml` — 32 AprilTag markers with (x, y, z, yaw) in map frame
- Registry includes wall tags (z=0.145m) and floor tags (z=0.002m)

## Dependencies

- apriltag_msgs, OpenCV (solvePnP), tf2, robot_localization (set_pose service)

## Improvement Opportunities

- Parameterize camera frame names (currently hardcoded `base_link`, `zed_left_camera_frame`)
- Add multi-tag voting or RANSAC for outlier rejection (currently single-marker corrections)
- Add camera_info subscription timeout (currently hangs silently if unavailable)
- Validate tag_size against registry to prevent silent misconfiguration
- Test on real hardware with isaac_ros_apriltag (only validated in simulation)
