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
- `/agv/collision_monitor_state` (std_msgs/String) — Nav2 safety chain.
- `/agv/mode/set` (std_msgs/String) — `nav | teleop | idle`.

## Invariantes

- The FSM lives in `include/agv_mode_arbiter/mode_fsm.hpp` (header-only,
  ROS-free). 16 unit tests cover every valid transition.
- Safety stop (`collision_monitor_state == "stop"`) overrides every state
  → `BLOCKED_HANDOFF` with `source=NONE` (zero cmd_vel).
- Operator `idle` / `teleop` override the FSM before zone-based logic.

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
