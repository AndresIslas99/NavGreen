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

## Relación con otros specs

- [specs/interfaces.yaml](../../specs/interfaces.yaml) — topics listed under
  `agv_markers` and future expansion to `agv_rail_approach`
- [specs/launch_sequence.yaml](../../specs/launch_sequence.yaml) — launches
  at t=7s with `enable_markers`
- [specs/persistence.yaml](../../specs/persistence.yaml) —
  `runtime_markers_registry` is the shared input file
