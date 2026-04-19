# Rail driver — spec (design document)

Status: **design only** (2026-04-18). No code exists yet. This file captures the
design decisions so the next session can implement without re-arguing them.

## Why rails need a separate driver

**Corrected greenhouse geometry — 20 rails total (operator-confirmed 2026-04-18)**:

The USD has **two independent rail sections**, front and rear, for a total of
20 rails (5 aisles × 2 rails × 2 sections). The old `world_config.yaml` only
documented the front section; the rear section was discovered when a test
run placed the robot in "west open" at x=0.9 and the sim showed the robot
standing on a rail tube.

All rails: 51 mm tube diameter, length 20 m, parallel to X-axis. Each aisle
has 2 rails at `y = y_aisle ± 0.225` (pair spacing 0.45 m).

### FRONT section `x ∈ [7.5, 27.5]`

| Aisle | Aisle center Y | Rail "−" Y | Rail "+" Y |
|---|---|---|---|
| 1 | −4.400 | −4.625 | −4.175 |
| 2 | −2.200 | −2.425 | −1.975 |
| 3 |  0.000 | −0.225 | +0.225 |
| 4 | +2.200 | +1.975 | +2.425 |
| 5 | +4.400 | +4.175 | +4.625 |

### REAR section `x ∈ [−16.5, 3.5]`

Same `(aisle, y_center)` mapping, different x range.

### THE only rail-free zone: `x ∈ (3.5, 7.5)` — the 4 m maneuvering gap

Between the rear end (x=3.5) and the front start (x=7.5), there is a 4-m
gap with no rails. This is the ONLY x-range inside the greenhouse where
the robot can traverse laterally (change aisles) without being surrounded
by rails. Plus the outer zones `x < −16.5` (far west) and `x > +27.5` (far
east) if the map extends there.

Robot track width 0.96 m > pair spacing 0.45 → the wheels go outside the
rails; the rail pair is between the wheels, under the chassis. The chassis
clears the 51 mm tubes vertically. But:

- Lateral clearance from robot centerline to each rail in an aisle: 0.48 m
  wheel half-span − 0.225 m rail = 0.255 m.
- If the robot yaws while inside an aisle, the wheels swing outward
  proportional to the wheel-base. Yaw error exceeding `atan(0.255/0.30)
  ≈ 40°` snags a wheel on a rail (0.30 m is approximate half-wheelbase).
- Contact with a rail under motion damages the tube OR the wheel.

Round 36 and 37 ALL failed with this root cause: waypoints labeled
"corridor" or "west open" were actually inside the REAR rail section.
Any MPPI attempt to rotate (to avoid phantom pointcloud obstacles, to
satisfy the Nav2 path planner's curves, to execute a yaw arc) while in
an aisle resulted in a wheel snagging a rail and the robot yawing out
of control.

### True "open corridor" waypoint zones

Only 3 x-ranges are guaranteed rail-free:

1. `x ∈ (3.5, 7.5)` — the inter-section gap (4 m wide)
2. `x < −16.5` — far-west, beyond the REAR rear-end (if map extends)
3. `x > +27.5` — far-east, beyond the FRONT front-end (if map extends)

For Phase 1 precision testing (15 cm mean gate), use zone 1 — the 4 m gap.
Traversals up to ~3 m fit here. Y-coordinate must respect crop row
clearances: usable y bands (same as before) are `(−5.0, −3.8) ∪ (−2.8,
−1.6) ∪ (−0.6, +0.6) ∪ (+1.6, +2.8) ∪ (+3.8, +5.0)`.

## ZED-based rail detection — new Phase 2 requirement

Operator directive 2026-04-18: the robot must detect rails visually through
the ZED camera and adapt behavior (stop rotation, hold longitudinal motion)
**without** relying on a pre-loaded map of rail positions.

Rationale:

1. Rails may be relocated between crops or seasons.
2. The greenhouse map (`greenhouse_v2`) already has drifts vs real rail
   positions; a purely static map plus inflation policy is not reliable.
3. The robot is already looking forward via ZED for other reasons (tags,
   crop inspection) — rail detection is a marginal add-on for already-live
   perception.

### Detection approach

Run a perception node (`rail_detector`, C++17, new package or
`agv_perception`) at 10 Hz that:

1. Subscribes to `/agv/zed/depth/depth_registered` (depth) and
   `/agv/zed/left/image_rect_color`.
2. Generates a bird's-eye-view (BEV) projection of the forward region
   (0.3 m – 3.0 m ahead, ±1.0 m lateral) from the depth image.
3. Detects rail candidates as long thin convex features on the floor
   plane (detect_feature_type = "tube", expected_diameter_m = 0.051).
   A RANSAC line fit with low residual (< 2 cm) is a rail hit.
4. Publishes `/agv/rail_detections` (`agv_msgs/RailDetections`) with
   per-detection `{id, start_xy_robot_frame, end_xy_robot_frame,
   confidence, diameter_m}`.
5. Merges the rail detections with the pose-based zone_detector (see
   below) to produce the final `/agv/zone/state` output. Priority:
   visual detection overrides pose lookup if confidence > 0.7 and two
   parallel rails are detected 0.45 m apart.

### Implementation priority

Phase 2 ordering:

1. **rail_driver node** (design complete, implementation pending) — operates
   on zone_detector output, regardless of detector source.
2. **zone_detector pose-based** (Phase 1 fallback, cheap) — already
   specced below.
3. **rail_detector ZED-based** (this section) — upgrades zone_detector
   precision and handles rail drift.
4. **AprilTag integration** — upgrades pose accuracy so zone_detector
   pose-based stays accurate over long sessions.

If rail_detector proves robust, zone_detector pose-based becomes a
fallback for when ZED is blocked/occluded.

In round 34c, Nav2's MPPI was used for rail waypoints (wp07, wp08, wp09).
Results:

- wp07 (2,0→5,0): RESET_TIMEOUT (never entered rail)
- wp08 (5,-2.2→7,-2.2): **err_xy=2.837 m, err_yaw=44°** — robot rotated
  in the rail, snagged on inflation boundary, ended 3 m off-course
- wp09 (5,2.2→7,2.2): RESET_TIMEOUT

The MPPI controller is free to sample angular commands and the
RotationShimController is free to engage at path discontinuities. Neither
knows the rail is a 1D channel. This is a classic case of the controller
being given a geometry it can't physically satisfy.

## Architectural decision

**Two operational modes**, mutually exclusive:

| Mode | Active nodes | cmd_vel source | Rotation |
|---|---|---|---|
| `corridor_nav` | Nav2 (bt_navigator + planner + controller + RotationShim + MPPI) | controller_server → velocity_smoother → collision_monitor → cmd_vel | Free |
| `rail_drive` | `rail_driver` (single C++ node) | rail_driver → velocity_smoother → collision_monitor → cmd_vel | **Forbidden (`wz=0`)** |

A `mode_arbiter` service decides which is active at any moment. Only one
publisher writes to `/agv/cmd_vel_raw` at a time; the other is deactivated
via Nav2 lifecycle (corridor_nav) or a gate flag (rail_drive).

## rail_driver node spec

### Responsibilities
- Accept a goal expressed as `{rail_axis_x: float, rail_y: float, goal_x: float, speed_max: float}` where `rail_axis_x` is the longitudinal axis direction in the `map` frame (usually +1 for forward along +X, -1 for -X).
- Publish `cmd_vel` (Twist) at 20 Hz:
  - `linear.x = speed_max * direction * (dist_to_goal > stop_band ? 1.0 : 0.0)`
  - `angular.z = 0.0` hard — never rotates
- Read pose from `/agv/odometry/global` (brain's ekf_global output in `map` frame)
- Declare success when `|current_x - goal_x| < stop_band` (~0.05 m)
- Publish `/agv/rail_driver/state` as `std_msgs/String` (JSON): `{"state": "idle|driving|reached|blocked", "err_x": float, "remaining_m": float}`
- Accept e-stop and collision_monitor interrupts through the same velocity_smoother chain as Nav2

### Interfaces
Actions:
- `/agv/drive_along_rail` (custom action, nav2_msgs/NavigateToPose-like wrapper): request contains `{rail_axis, rail_y, goal_x, speed_max, timeout_s}`; feedback `{remaining_m, current_speed}`; result `{final_x, final_y, err_xy, duration_s}`.

Topics:
- Sub: `/agv/odometry/global` (pose), `/agv/e_stop` (bool), `/agv/collision_monitor_state` (halt signal)
- Pub: `/agv/cmd_vel_raw` (Twist, ~20 Hz), `/agv/rail_driver/state` (String @ 5 Hz)

### Control law
Simple P-controller on remaining distance, forward-or-back:

```
err_x_rail = (goal_x - current_x) * rail_axis_sign
speed = clamp(kP * err_x_rail, -speed_max, speed_max)
if abs(err_x_rail) < stop_band_m: speed = 0
cmd_vel.linear.x = speed * abs(rail_axis_x)   # +X in base_link = forward
cmd_vel.angular.z = 0.0
```

Constants (initial, to be tuned):
- `kP = 1.0` (effective ~0.05 m/s per 5 cm remaining at start, ramping)
- `speed_max_rail = 1.0 m/s` commanded (5-20% slip in sim → ~0.05-0.2 m/s real)
- `stop_band_m = 0.05`
- `timeout_base_s = 60 + distance_m * 30` (conservative for 5% slip)

No deceleration phase needed — the stop_band + Nav2's collision_monitor handle the final approach.

## Zone detector — new sub-spec (2026-04-18)

Operator insight during round 36 testing: the robot was observed on a rail
*rotated* (not aligned with the rail longitudinal axis). Even with rail_driver
refusing to rotate, the robot can ENTER the rail zone already misaligned —
either because of imperfect teleport, EKF drift, or a corridor_nav exit that
didn't nail the heading before the zone transition.

**Conclusion**: we need an explicit `zone_detector` node with a published
state, and an alignment guard before rail_drive takes over.

### zone_detector node

Runs at 10 Hz. Publishes `/agv/zone/state` (std_msgs/String JSON):

```json
{"zone": "corridor|rail_y0|rail_yp22|rail_ym22|rail_yp44|rail_ym44|unknown",
 "confidence": 0.95,
 "rail_axis": [1.0, 0.0, 0.0],     // direction of rail if in rail zone
 "rail_offset_lat": -0.015,         // lateral offset from rail centerline (m)
 "rail_yaw_error": -0.087,          // yaw error vs rail axis (rad)
 "source": "pose|apriltag|sensor"}
```

Sources (priority order, later overrides earlier):

1. **Pose-based (Phase 1, pure geometry)**:
   - Reads `/agv/odometry/global`
   - Bucket lookup: if `3.6 <= x <= 7.4 AND abs(y - rail_y_candidate) < 0.15`,
     zone = matching rail; else `corridor`.
   - `confidence = 1.0 - clip(abs(y - rail_y_candidate) / 0.15, 0, 1)`.
   - This is the fallback when neither tags nor rail sensors exist.

2. **AprilTag-based (Phase 2)**:
   - Tag at each rail entry with ID encoding rail identity and axis.
   - When tag detected with `err_xy < 0.30`, override zone_detector with
     `source: "apriltag"` and `confidence: 0.99`.
   - On exit: second tag or dead-reckon timeout.

3. **Sensor-based (Phase 3, future hardware)**:
   - IR edge sensors on chassis sides detecting rail bumps/paint.
   - Magnetic/RFID markers on floor.
   - Downward camera detecting rail lines.
   - Override with `source: "sensor"` and `confidence: 1.0`.

### Alignment guard

Before mode_arbiter switches to `rail_drive`, it consults zone_detector:

- Required: `rail_yaw_error < 0.087 rad` (5°) AND `abs(rail_offset_lat) < 0.08 m` (8 cm).
- If violated: stay in `corridor_nav`, publish a goal that first ALIGNS the
  robot to the rail (rotate-in-place + lateral nudge via Nav2), THEN switch.
- Alignment goal: pose at `(rail_entry_x - 0.3, rail_y, atan2(rail_axis))`
  with tight tolerance (xy_tol=0.05, yaw_tol=0.05).
- Only after alignment confirmed does mode_arbiter call `rail_drive.activate(goal)`.

### In-rail attitude correction

Even after a clean entry, the robot can drift laterally in the rail (EKF
noise, wheel slip asymmetry). rail_driver monitors `rail_offset_lat` and
`rail_yaw_error` from zone_detector:

- If `abs(rail_yaw_error) < 0.05 rad (3°)`: drive straight, `wz = 0`.
- If `0.05 <= abs(rail_yaw_error) < 0.1 rad (3-6°)`: apply tiny corrective
  `wz = -kAlign * rail_yaw_error` with `kAlign = 0.3` and `abs(wz) <= 0.1 rad/s`.
  Forward vx is reduced to 70% during the correction.
- If `abs(rail_yaw_error) >= 0.1 rad (>6°)`: transition to `blocked_misaligned`,
  stop, request corridor_nav recovery. Do not try to rotate — at this angle the
  robot is already partially on a crop row.

This gives a rail-compatible "nudge" behavior without violating the spirit of
"no rotation on rails": tiny corrections are allowed within the physical
envelope the rail provides, but large corrections trigger handoff back to
corridor_nav.

## Transition mechanism (Phase 1: pose-based, before tags)

Until AprilTags are integrated (Phase 2), the mode arbiter uses absolute pose:

```
def current_mode(x, y):
    if 3.6 <= x <= 7.4:
        if y in (0, ±2.2, ±4.4):   # snap to rail centerlines
            return "rail_drive"
    return "corridor_nav"
```

The test harness calls `/agv/mode/set` before each waypoint to enter the right mode. Waypoint YAML gains a `mode` field:

```yaml
- id: wp08
  mode: rail_drive             # NEW
  rail_axis: [1, 0, 0]         # rail direction in map frame
  rail_y: -2.2                 # which rail
  start: {x: 5.0, y: -2.2, yaw: 0.0}
  goal:  {x: 7.0, y: -2.2, yaw: 0.0}
```

## Phase 2 upgrade (AprilTag-based trigger)

Replace pose-based mode detection with tag detection:

- Tag at rail entry (e.g., tag ID 7 at `(3.6, -2.2, 0.6)`) facing +X
- When brain sees tag 7 with `err_xy_to_tag < 0.3 m`, call `/agv/mode/set rail_drive`
- Exit: another tag at rail end, or fallback to dead-reckon when rail_driver reports `state=reached`

Integration path in Phase 2:
1. Add tags to USD (sim host)
2. Enable `agv_markers` in HIL launch (already packaged, just not enabled in HIL)
3. Mode arbiter consumes `/agv/marker_pose` to decide mode
4. Test harness switches to tag-relative goals: `goal_rel_tag: [2.0, 0.0, 0.0]` instead of absolute map coords

## Safety

- **`wz = 0` hard** in the node — no parameter to raise it. Protects against code bugs or bad goals.
- **Rail-exit detection**: if GT shows `|current_y - rail_y| > 0.15 m` (robot drifted laterally out of rail), abort with `state=blocked` and request corridor_nav takeover.

### Obstacle policy: STOP-AND-WAIT, never path-around

Field-observed rule (2026-04-18, operator-validated): on a rail, the ONLY
acceptable response to an obstacle is to STOP and WAIT for it to be removed.
Rotation or backing-away would damage crop rows on either side (0.55 m clearance
each side = less than the robot's 0.74 m width after inflation).

Implementation:

1. `rail_driver` subscribes to `/agv/collision_monitor_state` (String) AND watches
   its own local costmap if fed (future work).
2. When `collision_monitor_state == "stop"` (obstacle inside stop polygon):
   - Publish `cmd_vel.linear.x = 0.0, angular.z = 0.0` (hold in place)
   - State transitions to `blocked_wait`
   - Do NOT re-plan, do NOT call any recovery behavior, do NOT invoke behavior_server.
   - Keep publishing zero-velocity cmd_vel at the normal rate so the timeout-
     based watchdogs upstream (cmd_vel_gate) don't fire.
3. When `collision_monitor_state` returns to `clear`:
   - Resume `state=driving` and resume the P-controller on `err_x_rail`.
   - Optional: emit a `resume` event on `/agv/rail_driver/events`.
4. If `blocked_wait` persists beyond `AGV_RAIL_OBSTACLE_TIMEOUT_S` (default
   1800 s = 30 min), emit `state=blocked_timeout` and return action failure.
   Only after this does the operator have to take over manually.

Consequence: the Nav2 `behavior_server`'s `spin`/`backup`/`wait` plugins MUST be
deactivated for the duration of `rail_drive` mode. The `mode_arbiter` handles
this by calling the behavior_server lifecycle `deactivate` when entering
rail_drive, and `activate` again on return to corridor_nav.

### Corridor vs rail obstacle policy contrast

| Mode | Obstacle in stop_zone | Obstacle in slowdown_zone |
|---|---|---|
| `corridor_nav` | collision_monitor halts → BT invokes spin/wait/replan → Nav2 routes around | collision_monitor scales cmd_vel to 30% → MPPI proceeds cautiously |
| `rail_drive`   | **HOLD zero cmd_vel indefinitely** — no replan, no rotation, no backup | **HOLD zero cmd_vel** — even slowdown is interpreted as "too close, stop" on rails |

Rationale: the slowdown zone exists in corridor mode because obstacles can be
anticipated (MPPI sees them 25 cm ahead and scales down before they hit the stop
zone). On a rail, any obstacle within the slowdown band is already well inside
the 0.55 m lateral clearance and further approach would risk contact with the
obstacle OR with the crop row on either side. Stop is the only safe action.

## What this unlocks

- All 3 rail waypoints (wp07/wp08/wp09 from waypoints_20.yaml) can pass at 15 cm.
- Provides the backbone for Phase 2 tag-anchored precision.
- Separates rail-specific failures from MPPI-tune iteration in `docs/validation/iteration_loop.md`.

## Implementation estimate

- Node implementation: 1-2 days C++17
- Action interface + launch integration: half day
- Test harness waypoint YAML extension: half day
- Round-of-iteration on kP, stop_band: 1 day
- Phase 2 tag integration: separate sprint

No implementation in this session — this document is the handoff for the next one.
