# agv_rail_driver

Longitudinal-only drive controller. Takes a goal pose, publishes cmd_vel
with **`angular.z == 0`** hardcoded. Safe for rail aisles (where any rotation
risks hitting a 51 mm rail tube) and for the 4-m gap (where MPPI's
angular-velocity sampling was driving the robot into crop rows).

## Responsabilidades

- **Sí hace**: read goal + pose, compute P-controlled forward/backward linear
  velocity toward the goal along the robot's X axis; publish cmd_vel with
  `wz = 0` hard; expose a state machine (IDLE/DRIVING/REACHED/BLOCKED_*).
- **No hace**: path planning, rotation, obstacle avoidance via replanning.
  Obstacles are handled only through `collision_monitor_stop → hold
  indefinitely`. No rotation policy means any misalignment blocks motion;
  alignment is the responsibility of the corridor navigator before handoff.

## Interfaces propias

### Published

- `/agv/cmd_vel_rail` (geometry_msgs/Twist, 20 Hz): `linear.x` ∈ [-speed_max,
  speed_max], `angular.z == 0` always. `linear.y/z` unused.
- `/agv/rail_driver/state` (std_msgs/String, 20 Hz): JSON `{"state": str,
  "linear_x": float, "remaining_m": float, "in_rail_zone": bool,
  "collision_stop": bool}`. States are `idle | driving | reached |
  blocked_wait | blocked_misaligned | blocked_lateral`.

## Interfaces consumidas

- `/agv/odometry/global` (nav_msgs/Odometry) — from `ekf_global`.
- `/agv/rail_driver/goal` (geometry_msgs/PoseStamped) — target pose (x, y
  in `map`; yaw ignored).
- `/agv/zone/state` (std_msgs/String) — from `agv_zone_detector`; supplies
  `rail_yaw_error` and whether we're in a rail aisle.
- `/agv/collision_monitor_state` (nav2_msgs/CollisionMonitorState) — from
  Nav2's `collision_monitor`; takes absolute priority over all other inputs.
  Typed subscription since the Section-0 Day-2 field fix (2026-05-13); the
  earlier std_msgs/String subscription never matched the publisher and was
  silently dropped by DDS.
- `/agv/rail_detections` (geometry_msgs/PoseArray) — from
  `agv_rail_detector` (Stage K). Two poses in base_link; midpoint Y is the
  signed visual lateral offset, average yaw is the rail-axis direction.
- `/agv/rail_detector/state` (std_msgs/String JSON) — visual confidence
  used for gating the visual-vs-pose switch.

## Invariantes

- `angular.z == 0` in every single published message. Guaranteed by the
  node (defensive) and by `rail_controller.hpp` (structural). There is no
  parameter to raise it.
- `collision_monitor_stop → BLOCKED_WAIT` overrides everything else
  (including yaw/lateral aborts).
- The controller logic lives in `include/agv_rail_driver/rail_controller.hpp`
  as a ROS-free header, so unit tests can stress-cover it without spinning
  a node. See `test/test_rail_controller.cpp` (19 cases incl. visual-
  feedback preference, staleness fallback, low-confidence rejection).
- Visual inputs (`visual_lat_offset`, `visual_yaw_error`) override the
  pose-based lateral/yaw aborts only when `visual_confidence > 0.7` AND
  `visual_age_s < 0.5`. Otherwise the pose-based checks apply. Pose is
  always the safe default.

## Failure modes

- `BLOCKED_LATERAL` → robot drifted more than 0.30 m off the goal's y. Exits
  to corridor_nav in a future arbiter; for now the test harness catches this
  and reports as ABORTED.
- `BLOCKED_MISALIGNED` → yaw error vs rail axis exceeded `yaw_abort_rad`
  (default 15°) while in a rail aisle. Same exit: corridor_nav must align.
- `BLOCKED_WAIT` → collision_monitor sees an obstacle in the stop zone.
  Holds zero velocity indefinitely until obstacle clears (never rotates,
  never retries).

## Relación con otros specs

- `docs/validation/rail_driver_spec.md` — full design rationale + Phase 2
  integration plan.
- `specs/interfaces.yaml` — will register `/agv/cmd_vel_rail` and
  `/agv/rail_driver/state` on merge.
- The `mode_arbiter` (not yet implemented) subscribes to zone_detector and
  decides which of `{Nav2/controller_server, rail_driver}` publishes to the
  physical `/agv/cmd_vel_raw` at any given time.
