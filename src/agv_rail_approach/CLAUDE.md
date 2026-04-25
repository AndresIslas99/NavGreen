# agv_rail_approach

Precision docking controller that uses AprilTag detections to guide the
robot into a rail-aligned approach. Consumes `/agv/marker_pose` and
publishes velocity commands on `/agv/cmd_vel` during the approach phase.

## Responsabilidades

- Given a target AprilTag (configured via `runtime_registry_file` — a
  registry of `rail_start` tags), steer the robot so that its heading
  aligns with the rail direction before crossing the tag.
- Publish a state topic that the backend can display.
- Never drive autonomously outside an explicit "approach requested"
  context.

## Interfaces propias

- **Topic**: `/agv/cmd_vel` (`geometry_msgs/msg/Twist`) — velocity commands
  during an active approach.
  - **Warning**: `agv_rail_approach` shares `/agv/cmd_vel` with `teleop_server`.
    Only one publisher should command motion at a time. The backend's state
    machine must ensure mutually exclusive authorship.
- **Topic**: `/agv/rail_approach/status` (`std_msgs/msg/String`, JSON) —
  approach state (`idle`, `aligning`, `approaching`, `settled`, `failed`).

## Interfaces consumidas

- `/agv/marker_pose` — from `agv_markers::marker_correction`
- `/agv/odometry/global` — from `ekf_global`
- Runtime parameter `runtime_registry_file` — shared with
  `agv_markers::marker_correction`, contains the set of approved
  `rail_start` tag definitions.

## Invariantes

- During `settled` state, must not publish any cmd_vel (pass control back
  to the waypoint executor or teleop).
- Must honor the global e-stop: subscribe to `/agv/e_stop` and zero
  output immediately.
- Should never request a goal > 2 meters from the current robot pose
  (defense against runaway approaches).

## Failure modes

- If `marker_pose` goes stale (> 0.5s), transition to `failed`, publish
  zero cmd_vel, and let the supervisor recover.
- If the approach does not converge within a configurable timeout,
  report failure — do not keep trying.

## Status

- **Production readiness**: Partially integrated. Launched in
  `agv_full.launch.py` at t=7s conditional on `enable_markers=true`.
- **Known gaps**: No `TASK.yaml` before 2026-04-13 audit — now tracked.
  Sharing `/agv/cmd_vel` with teleop is a latent race hazard; should be
  resolved by routing through the dashboard state machine.

## Service contract — skip_coarse_approach (2026-04-25)

`agv_interfaces/srv/RailApproach` carries a boolean flag
`skip_coarse_approach`. Default `false` preserves the original two-phase
flow:

1. **COARSE_APPROACH**: rail_approach asks Nav2 to drive the robot to a
   standoff `coarse_standoff_distance` meters before the tag (in map
   frame). Requires `localization.action == 'LOCALIZED'`; the node
   rejects the service call otherwise.
2. **TAG_ACQUISITION**: wait for the camera to detect the target tag.
3. **FINE_SERVOING**: PI+FF iter-46 controller drives the robot to the
   commanded `(offset_x, offset_y)` of the tag in the camera frame.

When `skip_coarse_approach=true`, the node bypasses (1) entirely and
jumps straight to TAG_ACQUISITION. This is the correct path when:

- The robot is already physically in front of the tag (camera detection
  is fresh) AND
- Localization in the map frame is unreliable or unavailable (DEGRADED,
  FAILED, or no map loaded).

The mode_arbiter has a carve-out (2026-04-25) that activates
`Source::APPROACH` even when `operator_mode == "teleop"` while
rail_approach is in TAG_ACQUISITION or FINE_SERVOING. That makes the
skip-coarse path usable without first switching the operator mode pill
to nav.

Backend bridge: `POST /api/apriltags/:hw_id/align` is the operator-facing
endpoint that always sets `skip_coarse_approach=true`. The traditional
`POST /api/apriltags/:id/navigate` keeps the default false and goes
through Nav2.

## Relación con otros specs

- [specs/interfaces.yaml](../../specs/interfaces.yaml) — topics listed under
  `agv_markers` and future expansion to `agv_rail_approach`
- [specs/launch_sequence.yaml](../../specs/launch_sequence.yaml) — launches
  at t=7s with `enable_markers`
- [specs/persistence.yaml](../../specs/persistence.yaml) —
  `runtime_markers_registry` is the shared input file
