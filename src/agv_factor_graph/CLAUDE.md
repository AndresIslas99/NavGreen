# agv_factor_graph

Factor-graph based state estimator, running in parallel to `ekf_global`
for validation. Publishes `/agv/factor_graph/odometry` which can be
compared against `/agv/odometry/global` to assess whether a cutover from
EKF to factor-graph is justified.

## Responsabilidades

- Maintain a factor graph over wheel odometry, IMU, cuVSLAM, and marker
  poses. Run optimization at 10 Hz.
- Publish an independent pose estimate on `/agv/factor_graph/odometry`
  (NOT a TF publisher — `publish_tf=false`).
- Be a silent observer — this package does not participate in the
  operational TF tree today.

## Interfaces propias

- **Topic**: `/agv/factor_graph/odometry` (`nav_msgs/msg/Odometry`)
  — validation output only, not consumed by Nav2.

## Interfaces consumidas

- `/agv/wheel_odom` — from `agv_odrive`
- `/agv/imu/filtered` — from `agv_sensor_fusion::imu_filter`
- `/visual_slam/tracking/odometry` — from Isaac ROS cuVSLAM
- `/agv/marker_pose` — from `agv_markers::marker_correction`

## Invariantes

- `publish_tf` MUST remain `false` as long as the dual-EKF architecture
  owns TF. Enabling it would create a second publisher of `map→odom` and
  corrupt the state estimate. See [specs/state_machine.yaml](../../specs/state_machine.yaml)
  invariant `tf_map_odom_single_owner`.

## Failure modes

- If the optimizer diverges, the validation topic becomes noisy but no
  downstream consumer breaks (it is unsubscribed in production). Safe
  to ignore.
- If the node crashes, `ekf_global` continues to provide the authoritative
  pose. No navigation impact.

## Status

- **Production readiness**: Validation-only. Not in the critical path.
- **Known gaps**: No `TASK.yaml` before 2026-04-13 audit — now tracked.
  No integration tests.

## Relación con otros specs

- [specs/interfaces.yaml](../../specs/interfaces.yaml) — (factor_graph
  topics are currently out-of-scope for cross-package interface tracking,
  but should be added here when the cutover is planned)
- [specs/launch_sequence.yaml](../../specs/launch_sequence.yaml) —
  `factor_graph` entry at t=4.5s
- [specs/state_machine.yaml](../../specs/state_machine.yaml) — invariant
  `tf_map_odom_single_owner`

## References

- GTSAM as the underlying factor graph library.
- Parallel-mode rationale documented in commit `0f3820a feat: MPPI
  controller + GTSAM factor graph (parallel mode)`.
