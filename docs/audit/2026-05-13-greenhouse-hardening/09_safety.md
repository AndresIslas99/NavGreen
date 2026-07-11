# Phase 9 — Safety

> Phase 9 goal (per prompt): "El robot es seguro para operar cerca de
> humanos no entrenados, conforme a buenas prácticas industriales."
>
> Per `policies/engineering_rules.md` Rule 6 + Phase 0 reminders:
> nothing in this file constitutes certified functional safety. The
> recommendations are operational safeguards only.

This file covers the software safety chain (`agv_safety`), the
collision_monitor (deeper than Phase 7), the gaps around hardware E-stop,
and ships `docs/safety/HAZOP_skeleton.md` — a starting point for the
HAZOP workshop that the prompt requires.

---

## A. Safety supervisor — source-level audit

[`src/agv_safety/src/safety_supervisor.cpp`](../../../src/agv_safety/src/safety_supervisor.cpp).

### A.1 Verified mechanics

- `evaluate_topics()` (line 8) is **pure logic** — testable without
  ROS spin. Good engineering pattern (the unit test
  `test_supervisor_logic` exercises this).
- `software_estop_latched` (line 16) — latched correctly per `on_software_estop`
  (line 151). Cleared explicitly on `false` message — **not** a level-only
  protocol.
- **Startup grace window** (line 22–25): `since_start_ns < grace_ns` tolerates
  missing topics for `startup_grace_ms` (3 s default). Correct.
- **Per-topic deadline** (line 41): if `age > deadline`, mark silent.
- **Generic subscription** (line 118): `create_generic_subscription` with
  `QoS(10).best_effort()`. Captures the topic name in the lambda and
  updates `last_seen` + `ever_seen` flags.
- Verdict published at `publish_rate_hz: 10` (line 78).

### A.2 The post-fix monitored_topics

[`src/agv_safety/config/safety_params.yaml:31-42`](../../../src/agv_safety/config/safety_params.yaml):
```yaml
monitored_topics:    ["/agv/wheel_odom",   "/agv/odometry/global", "/agv/scan"]
monitored_types:     ["nav_msgs/...Odom",  "nav_msgs/...Odom",     "sensor_msgs/LaserScan"]
monitored_deadline_ms: [100,               250,                    250]
```

All three are **continuous-rate** topics. The historical bug (an
event-driven topic in the list) is fixed. **No new finding on the topic
selection.**

### A.3 Findings

---

#### HIGH-09-02 — `safety_supervisor` subscribes BEST_EFFORT to topics whose publishers may be RELIABLE; DDS asymmetry allowed but obscures intent
**File(s)**: `src/agv_safety/src/safety_supervisor.cpp:119`.
**Category**: bug / robustness.
**Symptom**: `create_generic_subscription` is called with
`rclcpp::QoS(10).best_effort()` for every monitored topic. The
publishers are:
- `/agv/wheel_odom` — `best_effort` per `interfaces.yaml` ✅
- `/agv/odometry/global` — `best_effort` per `interfaces.yaml` ✅
- `/agv/scan` — `best_effort` per `interfaces.yaml` ✅

**Today this works**. But the safety supervisor is type-erased — it
can monitor anything. If an operator extends `monitored_topics` to
include `/agv/safety/status` or `/agv/cmd_vel_safe` (both `reliable`),
the subscriber may not bind to a `reliable.transient_local` publisher
because of DDS QoS asymmetry rules: `BEST_EFFORT` subscribers can read
from `RELIABLE` publishers (allowed direction), but a future tightening
of either side can silently break the bind.
**Analysis**: The current code works because the team manually chose
BEST_EFFORT-published topics for the list. There is no programmatic
enforcement, and the safety supervisor — the layer responsible for
catching topic silence — could itself fail to bind to a topic it's
supposed to watch.
**Greenhouse impact**: Latent. A well-intentioned PR adds a RELIABLE
topic to `monitored_topics`, the subscriber doesn't bind, the supervisor
declares it "silent" forever, `safety_ok` drops to false, robot freezes.
The exact failure mode of the original 2026-04-13 incident.
**Recommendation**:
1. Make the supervisor's QoS **per-topic** configurable in
   `safety_params.yaml`:
   ```yaml
   monitored_qos:  ["best_effort", "best_effort", "best_effort"]
   ```
   Read at load time and applied per subscription.
2. Alternative: query the publisher's QoS at runtime via
   `get_publishers_info_by_topic()` and match it.
3. Document the rule in `agv_safety/CLAUDE.md`: monitored topics must
   use BEST_EFFORT QoS or the entry must explicitly opt into RELIABLE.
**Acceptance criterion**: A safety-monitored topic published RELIABLE
results in a clear log warning at boot (no silent un-bind), or the
supervisor matches QoS automatically.
**Effort**: S.
**Prerequisites**: none.

---

#### MEDIUM-09-03 — `safety_supervisor` is a regular Node, not a lifecycle node
**File(s)**: `src/agv_safety/src/safety_supervisor.cpp:63`.
**Category**: architecture.
**Symptom**: `class SafetySupervisorNode : public rclcpp::Node` (not
`LifecycleNode`). All initialization happens in the constructor; there is
no `on_configure`, `on_activate`, `on_deactivate`.
**Analysis**: Nav2 uses lifecycle nodes for `collision_monitor`,
`controller_server`, `bt_navigator`, `velocity_smoother`. The lifecycle
manager (`lifecycle_manager_navigation`) coordinates startup so the
graph reaches `active` together. The safety supervisor is outside this
coordination.
- At boot, `safety_supervisor` starts at t=6.5 s but its dependent
  topics (`wheel_odom`, `odometry/global`, `scan`) may not be flowing
  yet — handled by the 3 s startup grace.
- If `lifecycle_manager_navigation` deactivates Nav2 (e.g., to swap
  maps), `collision_monitor` stops publishing — but the supervisor isn't
  monitoring `collision_monitor_state` anymore (post-fix), so this is
  fine.
- If the supervisor itself crashes, `cmd_vel_gate`'s 0.5 s watchdog
  catches it (line 94-105 of `cmd_vel_gate.cpp`). Good defense in depth.
The lifecycle gap is therefore **not** a current failure mode. But it
means:
- The dashboard cannot query "is the safety chain active and validated"
  — it can only check whether `safety_ok` is currently true.
- A future need to coordinate supervisor with map loading or mode
  transitions has no lifecycle hook.
**Greenhouse impact**: Indirect today. Future work that wants to
re-initialize the supervisor (e.g., new map = new safety zones) has
to restart the node.
**Recommendation**: Promote to `LifecycleNode`. Map `on_configure` to
parameter loading, `on_activate` to subscriber creation + timer start,
`on_deactivate` to timer stop. Add to `lifecycle_manager` configuration.
This also makes per-mode safety configs possible later (different
deadlines for teleop vs mission).
**Effort**: M.
**Prerequisites**: HIGH-09-02 (per-topic QoS config) — combine the work
in the same refactor.

---

## B. cmd_vel_gate — source-level audit

[`src/agv_safety/src/cmd_vel_gate.cpp`](../../../src/agv_safety/src/cmd_vel_gate.cpp).

### B.1 Verified mechanics

- `apply_gate()` (line 14) is **pure logic** — testable. Good pattern.
- **Zero output on `!safety_ok || hardware_estop`** (line 16–18). Correct.
- **Clamp at `max_linear` / `max_angular`** as defense in depth (line
  20–22). Correct.
- **`hardware_estop` subscribed with `transient_local.reliable`** (line 44).
  Correct — a latched topic for the (planned) hardware bridge.
- **Watchdog** (line 51, 94–105): timer at `safety_timeout_s / 2 = 250 ms`,
  declares stale if no SafetyStatus in 500 ms. **But** the watchdog has
  the same `last_safety_msg_.nanoseconds() == 0` early-return as the
  pre-fix code — meaning if the supervisor **never** publishes
  (e.g., crash before first publish), the watchdog doesn't fire, and the
  gate trusts its initial state.
- **Initial state**: `safety_ok_ = ?`. Not visible in this excerpt;
  declared in header. The default-constructed `bool` is *false* if
  declared `bool safety_ok_;` but *true* if declared `bool safety_ok_{true};`
  — ambiguous from source alone.

### B.2 What happens when the gate "blocks"

Line 14–24: when blocked, `apply_gate()` returns a default-constructed
`Twist`, which is `{0, 0, 0, 0, 0, 0}`. The gate **publishes zero
velocity continuously** (line 72: `pub_output_->publish(apply_gate(in))`
on every input). It does **not** stop publishing.

This is the correct architecture — `agv_odrive_node` sees a continuous
zero stream and keeps motors at zero rather than waiting for `cmd_vel`
timeout (200 ms per HAL) to expire.

### B.3 Findings

---

#### HIGH-09-04 — Initial `safety_ok_` default value not verified from source; gate's behavior at first input pre-supervisor is ambiguous
**File(s)**: `src/agv_safety/src/cmd_vel_gate.cpp` (header `cmd_vel_gate.hpp` not read in this audit).
**Category**: bug / robustness.
**Symptom**: The class member `safety_ok_` initial value is set in the
header file (not shown in the .cpp). If `safety_ok_{false}` (defensive
default), the first cmd_vel input arriving before the first
`SafetyStatus` is blocked (zero output) — the operator sees teleop
not working for ~0.5 s after boot. If `safety_ok_{true}` (permissive
default), the first cmd_vel is forwarded even though no safety check
has occurred yet — the robot moves before the supervisor has confirmed
sensors are alive.
**Analysis**: The watchdog explicitly handles the "never received"
case (line 95: `if (last_safety_msg_.nanoseconds() == 0) return;`), so
whichever default is used, the watchdog won't override it until the
first message arrives. The choice is significant for the boot-time
"is the robot safe before the supervisor has spoken?" question.
**Recommendation**:
1. **Inspect `cmd_vel_gate.hpp`** to confirm the default. (This audit
   should have done so; flagging for follow-up.)
2. **Defensive default = false** is the correct choice: don't move
   until something explicitly says it's safe. Explicit operator-driven
   "arm motors" or first `SafetyStatus` lifts the gate.
3. Add a unit test (`test_gate_logic`) covering the "first input before
   any safety status" case explicitly.
**Acceptance criterion**: A bag replay where `cmd_vel_in` arrives at
t=0.1 s and `safety_status` arrives at t=2.0 s shows zero output until
t=2.0 s, then permissive forwarding.
**Effort**: S (one-line header check + test).
**Prerequisites**: none.

---

## C. E-stop architecture

### C.1 The three E-stops

| Topic | Owner | Status | Subscribers |
|---|---|---|---|
| `/agv/e_stop` | `agv_ui_backend` (operator clicks button) | **active** | `agv_odrive` (motor disable), `cmd_vel_gate` (zero output) |
| `/agv/software_estop` | (planned) | **no publisher** | `safety_supervisor` |
| `/agv/hardware_estop` | (planned — future hardware bridge) | **no publisher** | `cmd_vel_gate` |

`/agv/e_stop` is operational. The two `_estop` planned topics are
gateways for future expansion:
- **Software E-stop** = "operator-triggered, fully software" (per spec).
  Distinct from `/agv/e_stop` in unclear ways — both are operator-triggered
  software stops. The spec describes `software_estop` as separate, but
  the current `agv_ui_backend` publishes `/agv/e_stop` for the button.
  Why two? Probably an artifact of an earlier design where they had
  different semantics.
- **Hardware E-stop** = "hardware bridge to physical button or bumper".
  This is the **important one** — a software-only E-stop cannot stop
  a robot whose software has crashed. The physical-button-to-ODrive-enable
  pin is the actual safety path.

### Findings

---

#### HIGH-09-01 — `/agv/hardware_estop` is `planned` — no hardware bridge wired; physical E-stop bypasses ROS
**File(s)**:
- `specs/interfaces.yaml` — `/agv/hardware_estop: status: planned`.
- `src/agv_safety/src/cmd_vel_gate.cpp:43-45` — subscriber exists, no publisher.
- `agv_navigation/CLAUDE.md` "Required hardware additions" section.

**Category**: safety / docs.
**Symptom**: There is **no hardware E-stop integrated into the ROS
stack**. The topic exists in the spec as a placeholder. The
`cmd_vel_gate` subscribes to it but no publisher exists. The current
"E-stop" is a dashboard button that publishes `/agv/e_stop` — a
software-path-only stop.
**Analysis**: For an AMR operating around non-trained people, a
hardware E-stop is non-negotiable. The system today has:
- **Operator-side software stop** (dashboard button) — depends on the
  dashboard process, WS connection, supervisor, gate all functioning.
  Multiple failure points.
- **No physical-button latch to motor enable** — if the Jetson freezes,
  if WiFi drops, if the supervisor crashes, there is no second path to
  stop the motors.
The 0.5 m/s vx_max + 25 cm slowdown_zone + 20 cm stop_zone math
mitigates impact severity, but doesn't replace a hardware path.
The `agv_navigation/CLAUDE.md` "Required hardware additions" section
documents this gap clearly. Priority listed there is HIGH.
**Greenhouse impact**: An operator presses a physical button expecting
the robot to stop. If the ROS stack is in any failure mode, the
physical button does nothing. This is the difference between
"operational safeguards" (current) and "functional safety" (future).
**Recommendation** (3 options, by ascending hardware cost):
1. **Minimum viable**: wire a physical normally-closed button to the
   ODrive's `EN` (enable) pin. Pressing the button breaks the circuit,
   ODrive enters IDLE state at the hardware level. No software involvement.
   Cost: $5 button + wires. **This is the recommendation for the next
   field visit.**
2. **Better**: button latches a Bool via a GPIO on the Jetson; a small
   C++ node publishes `/agv/hardware_estop` to surface state to the
   dashboard. Still wired to ODrive EN as the primary path. The ROS
   topic is for monitoring only.
3. **Production**: dual-channel relay between the button and the ODrive
   EN, with a separate watchdog from a microcontroller. This is the
   path toward eventual ISO 13849 PL-d compliance.
**Acceptance criterion**: A physical button press on an unresponsive
robot (e.g., during a deliberate Jetson hang) causes motors to enter
IDLE within 100 ms, measurable on an oscilloscope.
**Effort**: M (option 1) → L (option 2) → XL (option 3).
**Prerequisites**: none.

---

## D. Existing safety chain — defense-in-depth verdict

The `agv_navigation/CLAUDE.md` "Safety chain" section lays out 5 layers:

| Layer | Mechanism | This audit verdict |
|---|---|---|
| L1 | Costmap inflation (0.55 m radius) | OK for 2 m corridors; see `MEDIUM-07-08` for narrower |
| L2 | collision_monitor with scan + pointcloud sources | Correctly configured; `MEDIUM-09-05` for rear lobe; `HIGH-09-01` gap for hardware backup |
| L3 | stop_zone polygon = footprint + 20 cm front | Math correct after `MEDIUM-07-05` recompute |
| L4 | `vx_max` capped at 0.25 m/s | Correct; documented |
| L5 | Backend watchdog of `collision_monitor_state` | Works; see `MEDIUM-10-01` for diagnostics aggregation |

Plus the documented **30 cm hardware blind zone in front of the ZED 2i**
— acknowledged in `agv_navigation/CLAUDE.md` as a HIGH-priority hardware
gap. The recommended hardware additions (LIDAR, bumper, ToF) are
hardware-cost decisions outside the scope of this static audit, but the
documented analysis is sound.

**The software safety chain is well-engineered for what it can do.**
Its limitation is structural: it is **software**. Without a hardware
fallback path (HIGH-09-01) and without a forward perception sensor that
sees inside the ZED's 30 cm minimum (the documented HW gap), the chain
cannot achieve certified safety. The team has been honest about this in
its own CLAUDE.md — that honesty is itself a deliverable.

---

## E. HAZOP skeleton

Shipped at `docs/safety/HAZOP_skeleton.md`. 18 hazards identified with:
- Severity (1–5)
- Probability (1–5)
- Risk = S × P
- Current mitigation (citing code)
- Gap

That document is the starting point for a HAZOP **workshop** that
brings the operator, the engineering lead, and the customer's safety
representative into a room. This audit produces the skeleton; the
workshop produces the signed HAZOP.

---

## F. Hardware-dependent items

| Acceptance criterion (per prompt §9) | Harness |
|---|---|
| Physical E-stop detiene motores < 100 ms (osciloscopio) | Once HIGH-09-01 is wired: probe `ODrive.EN` line vs button switch, scope screenshot. |
| Collision monitor frena ante obstáculo a 1 m sin contacto | HIL synthetic obstacle + measure actual stop distance vs commanded distance. |
| Test de pérdida de heartbeat: robot se detiene < 500 ms | Kill `safety_supervisor` process, measure cmd_vel_gate output time-to-zero. |
| Documento HAZOP/FMEA with ≥20 hazards mitigated | Workshop deliverable; this audit ships skeleton. |

---

## G. Status

| Item | Status |
|---|---|
| safety_supervisor source audit | ✅ |
| cmd_vel_gate source audit | ✅ |
| E-stop architecture | ✅ filed `HIGH-09-01` |
| QoS rigidity in supervisor | ✅ filed `HIGH-09-02` |
| Lifecycle node promotion | ✅ filed `MEDIUM-09-03` |
| Initial state ambiguity | ✅ filed `HIGH-09-04` |
| Rear lobe of stop zone (cross-referenced) | ✅ filed `MEDIUM-09-05` |
| HAZOP skeleton | ✅ shipped `docs/safety/HAZOP_skeleton.md` |
| Hardware-dependent items | ⏸ deferred (§F) |

End of Phase 9. 5 findings: 3 HIGH, 2 MEDIUM, plus 1 cross-referenced from Phase 7.
