# agv_safety

Software safety supervisor and final-stage cmd_vel gate for the AGV runtime stack.

## What this is NOT

This package implements **operational safeguards only**. Per
[policies/engineering_rules.md](../../policies/engineering_rules.md) Rule 6, it
must not be described as certified functional safety. Certified safety requires
hardware-integrated scope (safety scanners, safety PLC, dual-channel E-stop
relays) which is out of MVP scope.

## Nodes

### safety_supervisor_node
Subscribes to a configurable list of critical topics via type-erased
`GenericSubscription` and tracks their freshness. Publishes
`agv_interfaces/SafetyStatus` at a configurable rate (default 10 Hz). Any topic
that misses its deadline drops `safety_ok` to false. A latched
`software_estop` (Bool) input also forces `safety_ok=false`.

A startup grace window (`startup_grace_ms`, default 3000) tolerates topics that
have not yet been seen — without it the supervisor would falsely report
unsafe during the first few seconds of bringup.

**Which topics are valid to monitor** — the supervisor's `GenericSubscription`
works for any topic **that publishes continuously at a predictable rate**.
Event-driven topics (published only when something happens) will be marked
silent forever in the absence of events, which is NOT what freshness-based
watchdogs are meant to catch.

A concrete example of what NOT to monitor: Nav2's
`/agv/collision_monitor_state`. Nav2's collision_monitor only publishes this
topic when it processes a `cmd_vel_smoothed` input. In teleop-at-rest there
is no cmd_vel flowing through the Nav2 chain, so the topic stays silent, and
a freshness watchdog would falsely conclude the safety chain is dead. This
was the root cause of the 2026-04-13 teleop-broken incident: the supervisor
had `/agv/collision_monitor_state` in its `monitored_topics` list, which
silently gated all cmd_vel for minutes at a time. Discovered in Fase 6 bug
#1. collision_monitor liveness is instead verified by
`agv_healthcheck.sh` (at boot) and by the backend goal-dispatch watchdog
in `agv_ui_backend/src/index.ts`.

### cmd_vel_gate_node
Final stage of the cmd_vel pipeline. Subscribes to:
- `cmd_vel_in` — upstream cmd_vel (typically from `collision_monitor`)
- `safety_status` — `SafetyStatus` from `safety_supervisor_node`
- `hardware_estop` — `Bool` from any hardware E-stop bridge

Publishes `cmd_vel_out` (`Twist`). When `safety_ok` is false OR
`hardware_estop` is true, output is forced to zero. Otherwise the input is
clamped to `max_linear` and `max_angular` and forwarded.

A built-in watchdog also forces `safety_ok=false` if no `SafetyStatus` arrives
within `safety_timeout_s` (default 0.5s) — this catches a crashed
`safety_supervisor`.

## QoS contract for E-stop inputs

The `hardware_estop` subscription in `cmd_vel_gate_node` and the
`software_estop` subscription in `safety_supervisor_node` both use
`reliability: reliable` + `durability: transient_local`, so a late-joining
node still receives a latched E-stop published before it started.

DDS QoS matching **rejects** a `volatile` (default-durability) publisher
against a `transient_local` subscription: a future E-stop bridge that
publishes `std_msgs/Bool` with default QoS would never connect, and the
E-stop would be silently ignored. Any publisher on `/agv/hardware_estop` or
`/agv/software_estop` MUST use `reliability: reliable` +
`durability: transient_local`. This durability requirement belongs in the
`qos:` blocks of both topics in `specs/interfaces.yaml`, which currently
records neither.

## Topology (wired into agv_full.launch.py as of 2026-04-13)

```
Nav2 cmd_vel
  -> velocity_smoother
  -> collision_monitor
  -> /agv/cmd_vel_collision_safe   (renamed from cmd_vel_safe)
  -> cmd_vel_gate
  -> /agv/cmd_vel_safe              (consumed by odrive_can_node)
```

The rename of the collision_monitor output topic from `cmd_vel_safe` to
`cmd_vel_collision_safe` was completed during the reboot-to-production plan
of 2026-04. Both the YAML and `agv_full.launch.py` are now consistent:

1. `agv_navigation/config/collision_monitor.yaml` — `cmd_vel_out_topic: cmd_vel_collision_safe`
2. `agv_full.launch.py:317-335` — includes `safety.launch.py` at t=6.5s with `IfCondition(has_map)`
3. `cmd_vel_gate` remaps `cmd_vel_in → cmd_vel_collision_safe`, `cmd_vel_out → cmd_vel_safe`
4. `agv_odrive_node` subscribes to `cmd_vel_safe` when `has_map=true`

The 2026-04-13 audit initially misdiagnosed the teleop-broken bug as a
watchdog timing issue. The actual root cause was `monitored_topics` including
an event-driven Nav2 topic that never published at rest — see the
"Which topics are valid to monitor" note above.

## Configuration

[config/safety_params.yaml](config/safety_params.yaml) — supervisor monitored
topics + gate clamps. The three `monitored_*` arrays are zipped element-wise
and must be the same length.

## Testing

```
colcon test --packages-select agv_safety --event-handlers console_direct+
```

Two unit tests:
- `test_supervisor_logic` — pure logic (no node spin) for the freshness verdict
- `test_gate_logic` — pure logic for the gate's clamp + zero rules

## Improvement opportunities

- Lifecycle node variant (currently a regular Node)
- Latched diagnostics so the dashboard can show *why* safety dropped
- Per-topic QoS profiles (currently best_effort for everything)
- Heartbeat protocol so each critical node publishes its own liveness header,
  decoupling the supervisor from real data topics
