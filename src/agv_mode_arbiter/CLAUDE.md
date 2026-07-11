# agv_mode_arbiter

Owns `/agv/cmd_vel` publication in the 3-mode navigation architecture. At
any instant, exactly one upstream controller drives the robot; the arbiter
relays its Twist downstream and publishes the current mode.

## Responsabilidades

- **Sí hace**: consume zone, rail_approach/rail_driver state, safety stop,
  operator mode; step a pure FSM; relay the selected upstream Twist to
  `/agv/cmd_vel`; publish `/agv/mode/state`.
- **No hace**: any control math. Does not compute velocities; only picks
  between upstream sources. Does not manage Nav2 lifecycle — the
  controllers keep running; only their Twist is or is not relayed.

## Interfaces propias

### Published

- `/agv/cmd_vel` (geometry_msgs/Twist, 20 Hz): relay from selected source,
  or zero Twist when the source is `NONE`.
- `/agv/mode/state` (std_msgs/String, 20 Hz): JSON `{"mode", "source",
  "zone", "operator_mode", "transitions"}`.

### Subscribed

- `/agv/cmd_vel_nav`, `/agv/cmd_vel_approach`, `/agv/cmd_vel_rail`
  (geometry_msgs/Twist) — one of these is relayed.
- `/agv/zone/state` (std_msgs/String) — from `agv_zone_detector`.
- `/agv/rail_approach/state`, `/agv/rail_driver/state` (std_msgs/String).
- `/agv/collision_monitor_state` — subscribed **twice**, with two types:
  - `nav2_msgs/CollisionMonitorState` — Nav2's collision_monitor
    (production safety chain; `action_type == STOP` forces the stop).
  - `std_msgs/String` (`"stop"/"slowdown"/"clear"`) — HIL-only
    side-channel from `agv_hil_bridges/sim_obstacle_relay`.
  The two sources are OR-ed each tick; either one saying stop forces
  `BLOCKED_HANDOFF`.
- `/agv/mode/set` (std_msgs/String) — `nav | teleop | idle`.

## Invariantes

- The FSM lives in `include/agv_mode_arbiter/mode_fsm.hpp` (header-only,
  ROS-free). 22 unit tests cover every valid transition.
- Safety stop (Nav2 `CollisionMonitorState.action_type == STOP`, or the
  HIL String channel saying `"stop"`) overrides every state
  → `BLOCKED_HANDOFF` with `source=NONE` (zero cmd_vel).
- Operator `idle` / `teleop` override the FSM before zone-based logic.
- **RAIL_EXIT hard-lock**: once inside a rail (RAIL_DRIVE or RAIL_EXIT), the
  FSM never hands back to Nav2 until the robot is physically out of the
  rail + approach zones AND ≥ 1 m past the exit AprilTag (Stage M). This
  prevents MPPI from sampling rotations while the robot is inside the
  51 mm rail tubes or within reach of the crop rows flanking the tag.
  rail_driver's `wz == 0` hard-lock stays in charge until the release
  condition is met.
- Inside a rail, **reverse is allowed** (rail_driver can command
  `linear.x < 0` to back out). Outside a rail, Nav2's MPPI is configured
  with `vx_min: 0` so reverse is forbidden in corridor zones for safety.

## Failure modes

- If an upstream source stops publishing, the arbiter publishes zero
  Twist once the source's last message is older than
  `cmd_vel_source_timeout_ms` (default 250 ms; Sprint A.5 /
  HIGH-11-A-02) and WARNs, instead of relaying the stale cache at
  20 Hz. ODrive's own `cmd_vel_timeout_ms` (200 ms) remains the
  downstream backstop if the arbiter itself dies. Upstream controllers
  publishing zero on idle (rail_driver's IDLE state) is still good
  practice but no longer the only line of defense.
- `/agv/mode/set` missing is non-fatal: `operator_mode` defaults to
  `"nav"` so the FSM runs its zone-based logic.

## Relación con otros specs

- `specs/state_machine.yaml` layer 3 (`runtime_mode_arbiter` block to be
  added in P2.S7).
- `specs/interfaces.yaml` — registers `/agv/cmd_vel_nav`,
  `/agv/cmd_vel_approach`, `/agv/mode/state`, `/agv/mode/set`.
- `docs/validation/rail_driver_spec.md` — full design rationale.
