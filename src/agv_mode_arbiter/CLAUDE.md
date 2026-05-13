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
- `/agv/collision_monitor_state` (nav2_msgs/CollisionMonitorState) — Nav2 safety chain.
  Sprint A.5 / CRITICAL-11-A-01: subscriber was previously `std_msgs/String`,
  silently dropped by DDS due to type mismatch. Now uses the proper Nav2 type
  and compares `action_type` to `CollisionMonitorState::STOP` constant.
- `/agv/mode/set` (std_msgs/String) — `nav | teleop | idle`.

## Invariantes

- The FSM lives in `include/agv_mode_arbiter/mode_fsm.hpp` (header-only,
  ROS-free). 22 unit tests cover every valid transition.
- Safety stop (`CollisionMonitorState::action_type == STOP`) overrides every
  state → `BLOCKED_HANDOFF` with `source=NONE` (zero cmd_vel).
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

- If an upstream source stops publishing, the arbiter keeps relaying the
  last message until the FSM leaves that source. To hold zero velocity in
  that case, upstream controllers must publish zero themselves when idle
  (rail_driver does this via its IDLE state; Nav2 publishes nothing when
  no goal is active, which is fine because the FSM will only select
  `Source::NAV` while a Nav2 goal is being pursued).
- `/agv/mode/set` missing is non-fatal: `operator_mode` defaults to
  `"nav"` so the FSM runs its zone-based logic.

## Relación con otros specs

- `specs/state_machine.yaml` layer 3 (`runtime_mode_arbiter` block to be
  added in P2.S7).
- `specs/interfaces.yaml` — registers `/agv/cmd_vel_nav`,
  `/agv/cmd_vel_approach`, `/agv/mode/state`, `/agv/mode/set`.
- `docs/validation/rail_driver_spec.md` — full design rationale.
