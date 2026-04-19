# agv_rail_detector

ZED-depth → BEV → RANSAC rail tube pair detector. Publishes
`/agv/rail_detections` so `agv_rail_driver` can correct lateral drift
inside a rail aisle with visual feedback, not only pose.

## Responsabilidades

- **Sí hace**: re-project the ZED depth image into a ground-plane 2-D
  slice in `base_link`, run RANSAC to find a parallel pair of tube
  lines 0.45 m apart, publish two poses (one per rail line) with
  orientation set along the rail direction.
- **No hace**: any control. Never drives the robot. Consumers decide how
  to interpret stale or low-confidence detections.

## Interfaces propias

### Published
- `/agv/rail_detections` (geometry_msgs/PoseArray, 5 Hz): two poses per
  tick, positions at each rail's nearest point to the robot, orientations
  along the rail axis. Frame id: `base_link`.
- `/agv/rail_detector/state` (std_msgs/String JSON, 5 Hz): confidence,
  inlier counts, total BEV points, `has_detection`.

### Subscribed
- `/agv/zed/depth/depth_registered` (sensor_msgs/Image, 32FC1).
- `/agv/zed/left/camera_info` (sensor_msgs/CameraInfo).

## Invariantes

- `detect_rails(...)` in `rail_ransac.hpp` is ROS-free and deterministic
  (seeded xorshift RNG). 9 gtests cover empty/rotated/wrong-spacing/
  single-rail/outlier cases.
- Camera offset from base_link is read from params (matches
  `agv_description/config/robot_params.yaml` 2026-04-18 remeasurement).
  No tf2 runtime dependency.

## Failure modes

- No `camera_info` received: node starts but emits no detections
  (`has_detection: false`, silent on /agv/rail_detections).
- Depth encoding is not `32FC1`: same as above (silent).
- RANSAC fails to find a pair (low inliers, wrong spacing, single rail):
  `confidence=0` → consumers fall back to pose-based alignment.

## Relación con otros specs

- `specs/interfaces.yaml` — will register `/agv/rail_detections` and
  `/agv/rail_detector/state` on merge.
- `docs/validation/rail_driver_spec.md` — Phase 2 Stage J design.
- `src/agv_rail_driver` — consumer (Stage K).
