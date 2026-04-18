# agv_zone_detector

Publishes the current greenhouse zone (corridor vs rail aisle) by classifying
the robot's pose against the known greenhouse geometry. Feeds the mode_arbiter
and the `rail_driver` so Nav2 is not used inside rail aisles.

## Responsabilidades

- **Sí hace**: classify (x, y, yaw) → zone label + confidence + rail offsets.
- **No hace**: any control. Does not command cmd_vel. Does not switch modes.
  Other nodes (mode_arbiter, rail_driver, planners) decide what to do with
  the zone state.

## Interfaces propias

### Published

- `/agv/zone/state` (std_msgs/String, 10 Hz): JSON with:
  - `zone`: one of `corridor_west | corridor_east | gap | rail_aisle_0 |
    rail_aisle_p22 | rail_aisle_m22 | rail_aisle_p44 | rail_aisle_m44 | unknown`
  - `section`: `REAR | GAP | FRONT | OUTSIDE`
  - `aisle_y_center`: y of the matched aisle (null if not in rail)
  - `rail_offset_lat`: signed lateral offset from aisle center (m, null if not in rail)
  - `rail_yaw_error`: yaw vs rail +X axis, wrapped to [-π, π] (null if not in rail)
  - `confidence`: 0..1 (1.0 at aisle center, tapers linearly to 0.2 at aisle_half_width edge)
  - `source`: `pose` (Phase 1). Phase 2 will add `apriltag` and `zed_visual`.

## Interfaces consumidas

- `/agv/odometry/global` (nav_msgs/Odometry, `map` frame). From `ekf_global`
  in `agv_sensor_fusion`.

## Invariantes

- Zone is purely geometric lookup: no sensor, no visual, no history.
- Classification function (`classify()` in `zone_classifier_impl.hpp`) is
  header-only and ROS-free so it can be unit-tested without a spinning node.
- `rail_yaw_error` is in robot-frame-vs-rail-axis convention: 0 means robot
  faces +X along the rail; positive means robot rotated CCW.

## Failure modes

- If `/agv/odometry/global` stops publishing, zone_detector's timer still
  fires but does not publish (no last message). Downstream consumers see
  the topic go silent; they should handle stale state themselves.
- If the robot is in a rail section but between aisles (e.g., on top of a
  crop row at y=-1.1), zone is `unknown` with `confidence=0`. This is an
  error state that should never occur in normal operation.

## Geometry (hardcoded, matches USD)

- REAR rail section: x ∈ [-16.5, 3.5]
- GAP (rail-free):   x ∈ ( 3.5,  7.5)
- FRONT rail section: x ∈ [7.5, 27.5]
- Aisles in each section: y_center ∈ {-4.4, -2.2, 0, +2.2, +4.4}

Source of truth: `docs/validation/rail_driver_spec.md` (operator-confirmed
2026-04-18) and the sim `build_greenhouse_usd.py`.

## Relación con otros specs

- `specs/interfaces.yaml`: `/agv/zone/state` will be declared there on merge.
- `specs/state_machine.yaml`: the mode_arbiter transitions `corridor_nav ↔
  rail_drive` will consume this topic.
- Phase 2 upgrades:
  - AprilTag source: overrides when tag detected with high confidence.
  - ZED visual source: overrides when rail pair is RANSAC-fit in BEV depth.
