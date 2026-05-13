# Phase 11 — HMI, Commissioning, and Safety-Critical Orchestration

> Phase 11 goal (per prompt): close the audit on the cmd_vel multiplexer,
> the mission executor, the operator backend, and the frontend dashboard.
> Same level of scrutiny as Phase 9 for the safety-relevant pieces
> (`agv_mode_arbiter`, `agv_waypoint_manager`); structural-only for the
> UX-heavy frontend pieces (no claims about aesthetics, accessibility, or
> usability without a real user study).

This file delivers four sub-audits in one document plus a separate
commissioning walkthrough at `11_commissioning_walkthrough.md`.

Two findings rise to **CRITICAL** severity because they have direct
physical-operation or "any LAN device can drive the robot" implications.
They warrant a dedicated **Sprint A.5** before any other Phase 11 work.

---

## 11.A — `agv_mode_arbiter`: the cmd_vel multiplexer

The arbiter owns `/agv/cmd_vel`. At any instant exactly one upstream
source (NAV / APPROACH / RAIL / NONE) is selected by the FSM
[`mode_fsm.hpp:137`](../../../src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp).
The FSM itself is **pure and well-tested** (37 unit tests in
`test_mode_fsm.cpp` cover every documented transition, including all
operator-override carve-outs, rail-exit clearance gates, and
auto-approach gating). The findings here are not in the FSM but in
**the ROS-glue layer that feeds the FSM and acts on its outputs**.

### FSM verified transition matrix (condensed)

| From | Trigger | To | Source | Notes |
|---|---|---|---|---|
| any | `operator_mode == "idle"` | IDLE | NONE | Top-priority override |
| any | `operator_mode == "teleop"` AND `rail_approach_state in {"driving","tag_acquisition"}` | RAIL_APPROACH_ACTIVE | APPROACH | **Carve-out — see HIGH-11-A-03** |
| any | `operator_mode == "teleop"` AND not above | TELEOP | NONE | Teleop server publishes direct |
| any | `safety_stop == true` | BLOCKED_HANDOFF | NONE | **See CRITICAL-11-A-01 — currently unreachable** |
| CORRIDOR_NAV | `rail_driver_state == "driving"` | RAIL_DRIVE | RAIL | External-dispatch shortcut |
| CORRIDOR_NAV | `rail_approach_state == "driving"` | RAIL_APPROACH_ACTIVE | APPROACH | |
| CORRIDOR_NAV | `auto_approach && is_approach_zone()` | RAIL_APPROACH_PEND | NAV | + `request_rail_approach` |
| RAIL_APPROACH_PEND | `rail_approach_state == "driving"` | RAIL_APPROACH_ACTIVE | APPROACH | |
| RAIL_APPROACH_PEND | `rail_approach_state == "aborted"` | CORRIDOR_NAV | NAV | |
| RAIL_APPROACH_ACTIVE | `rail_approach_state == "settled"` | RAIL_DRIVE | RAIL | + `request_rail_drive_goal` |
| RAIL_DRIVE | `rail_driver_state == "reached"` | RAIL_EXIT | RAIL | + `request_rail_exit_push` |
| RAIL_DRIVE | `rail_driver_state in {"blocked_lateral","blocked_misaligned"}` | RAIL_EXIT | RAIL | Stay on RAIL — Nav2 must NOT take over inside aisle |
| RAIL_EXIT | `!rail_zone && !approach_zone && clearance ≥ 1.0 && rail_driver != "driving"` | CORRIDOR_NAV | NAV | Primary release |
| RAIL_EXIT | `approach_zone && rail_driver == "idle"` | CORRIDOR_NAV | NAV | Aux release for parked-in-approach trap |
| BLOCKED_HANDOFF | `safety_stop == false` | CORRIDOR_NAV | NAV | |

### Verified invariants (FSM-pure level)

- **Single source at a time**: the FSM produces exactly one
  `active_source` per `step()`. The node relays exactly that source in
  `on_tick()` ([mode_arbiter_node.cpp:329-344](../../../src/agv_mode_arbiter/src/mode_arbiter_node.cpp)).
- **Safety stop priority**: `safety_stop=true` in the FSM input forces
  `BLOCKED_HANDOFF` from any state ([mode_fsm.hpp:169-173](../../../src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp)).
  Test `SafetyStopOverridesEverything` ([test_mode_fsm.cpp:268-281](../../../src/agv_mode_arbiter/test/test_mode_fsm.cpp)) exercises this from 5 modes.
- **Rail exit hard-lock**: inside rail zones, neither "reached" nor
  "idle" releases to Nav2 without ≥1 m geometric clearance. Tests
  `RailExitHoldsInsideRailZone*` ([test_mode_fsm.cpp:154-265](../../../src/agv_mode_arbiter/test/test_mode_fsm.cpp)) exercise this.

### Findings

---

#### CRITICAL-11-A-01 — `collision_monitor_state` type mismatch makes `safety_stop` unreachable in the arbiter
**File(s)**:
- `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:171-175` — subscribes as `std_msgs::msg::String`.
- `specs/interfaces.yaml#/agv/collision_monitor_state` — declares the topic as `nav2_msgs/msg/CollisionMonitorState`.

**Category**: bug / safety.
**Symptom**: The arbiter subscribes to `/agv/collision_monitor_state`
with a `std_msgs/String` callback, but Nav2's `collision_monitor`
publishes `nav2_msgs/msg/CollisionMonitorState`. DDS type negotiation
fails silently — the subscriber **never receives a message**.
Consequence: `latest_inputs_.safety_stop` stays `false` forever, the
FSM never enters `BLOCKED_HANDOFF`, and the arbiter keeps relaying
whatever source it last selected even when Nav2 collision_monitor is
firing a `STOP` action.
**Analysis**: This is **the same root-cause class** as the 2026-04-13
audit bug #1 (`safety_supervisor` had collision_monitor_state in
`monitored_topics` with the wrong type). That bug was fixed by removing
the topic from the supervisor's list. The fix did **not** propagate to
the arbiter, which was created later in Phase 2 and copied the old
type. The string-match heuristic at line 174
(`msg->data.find("stop") != std::string::npos`) is moot because
`msg->data` never gets populated — there is no message.

The PHYSICAL safety chain is still intact: Nav2's collision_monitor
itself zeros `cmd_vel_collision_safe` when it fires, and the
`cmd_vel_gate` downstream forces zero on its own watchdog. So the
robot DOES stop. What's broken is the arbiter's **awareness**: the
FSM thinks the system is in CORRIDOR_NAV / RAIL_DRIVE / whatever, the
`/agv/mode/state` topic broadcasts "normal" mode, and the dashboard
shows green. **Operator sees no warning that a safety stop is in
progress.**
**Greenhouse impact**: Diagnostic and HMI integrity. The operator
cannot tell, from the dashboard, whether the robot is moving or has
been stopped by collision_monitor — both look the same in the mode
pill. In a busy greenhouse this delays operator response to the
underlying cause (person in path, sensor occlusion, plant intrusion).
**Benchmark**: Nav2's own docs and the 2026-04-13 audit fix already
established the correct type. Cross-component subscribers must match
the publisher's IDL.
**Recommendation**:
1. Change the subscription type to `nav2_msgs::msg::CollisionMonitorState`
   (include `<nav2_msgs/msg/collision_monitor_state.hpp>`).
2. Read `msg.action_type` (`int`) and compare to
   `nav2_msgs::msg::CollisionMonitorState::STOP` (a constant in the
   msg). Drop the string-match heuristic.
3. Add a `verify_interfaces.py` cross-check that flags any subscriber
   whose declared type differs from `specs/interfaces.yaml`.
4. Add a unit test that constructs a CollisionMonitorState message
   with `action_type=STOP` and confirms `safety_stop` flips.
**Acceptance criterion**: HIL scenario "synthetic obstacle entering
stop_zone" produces `/agv/mode/state` with `mode="blocked_handoff"`
within 200 ms.
**Effort**: S (1–2 h fix + test). Belongs to **Sprint A.5**.
**Prerequisites**: none.

---

#### HIGH-11-A-02 — No source-staleness timeout; stale `last_*` Twist relayed indefinitely
**File(s)**: `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:152-160` (sub callbacks), `:329-344` (relay).
**Category**: bug / failure mode.
**Symptom**: The arbiter caches `last_nav_`, `last_approach_`,
`last_rail_` as `ConstSharedPtr` and replaces them only when a new
message arrives. If the active source's publisher crashes
mid-traversal, the cached Twist is relayed at 20 Hz until the FSM
leaves that source.
**Analysis**: `agv_mode_arbiter/CLAUDE.md`'s "Failure modes" section
acknowledges this: "If an upstream source stops publishing, the arbiter
keeps relaying the last message until the FSM leaves that source. To
hold zero velocity in that case, upstream controllers must publish zero
themselves when idle." The mitigation is:
- ODrive's `cmd_vel_timeout_ms: 200` zeros wheel velocity if no
  `/agv/cmd_vel_safe` for 200 ms downstream of the smoother chain
  (Phase 2 `MEDIUM-02-07` context).
- Upstream controllers publishing zero on idle.

This works when controllers crash *cleanly* (publication stops →
downstream timeout catches it). But if a controller crashes while
sending a non-zero command (likely scenario: SIGABRT mid-callback),
the **last non-zero Twist is the cached value**. The arbiter relays
it for **up to 200 ms before ODrive's timeout zeros the wheels** — and
during those 200 ms at vx_max 0.25 m/s, the robot travels up to 5 cm
toward the unintended target.

For an FSM that's supposed to be the single-source authority on the
cmd_vel chain, having the *FSM itself* lack a per-source freshness
gate is structurally wrong. The fix is local and cheap.
**Greenhouse impact**: low-probability but real. 5 cm of unintended
motion at 0.25 m/s in a 1.0 m corridor is enough to scrape a plant
cuna or contact a maintenance worker.
**Benchmark**: Nav2's `velocity_smoother` itself has a freshness
timeout. Apollo Cyber RT controllers stamp every output with a wall
clock and consumers reject messages older than a configurable budget.
**Recommendation**:
1. Add `cmd_vel_source_timeout_ms` parameter (default 250 ms — 5×
   tick period at 20 Hz).
2. In `on_tick()`, before relaying, check the age of `last_<source>_`
   against `now()`. If older than the timeout, publish zero Twist and
   emit a throttled WARN log.
3. Track source freshness independently for the three sources so the
   FSM can react: a source going stale while it's the active one
   should bias the FSM toward NONE / IDLE.
**Acceptance criterion**: Kill rail_driver via SIGKILL while
RAIL_DRIVE is active; observe arbiter publishes zero Twist within
250 ms (measured against `/agv/cmd_vel`).
**Effort**: S.
**Prerequisites**: none.

---

#### HIGH-11-A-03 — TELEOP override carve-out allows `rail_approach` to drive while operator picked teleop
**File(s)**: `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp:147-161`.
**Category**: bug / UX expectation mismatch.
**Symptom**: When the operator publishes `/agv/mode/set teleop` AND
`rail_approach_state` is `"driving"` or `"tag_acquisition"`, the FSM
ignores the teleop directive and yields `Mode::RAIL_APPROACH_ACTIVE`
with `Source::APPROACH`. Operator believes they have taken control;
the robot continues executing rail_approach motion until the alignment
loop finishes or the operator escalates.
**Analysis**: The carve-out is **deliberate**, documented in code as
"2026-04-25 pure-alignment carve-out for `skip_coarse_approach=true`
workflow". Tests
`TeleopWithRailApproachActiveYieldsApproachSource` and
`TeleopWithRailApproachAcquiringYieldsApproachSource`
([test_mode_fsm.cpp:311-336](../../../src/agv_mode_arbiter/test/test_mode_fsm.cpp)) confirm the design.

But this conflicts with the operator's mental model of "TELEOP is my
control". Two cases to disambiguate:
- (a) Operator pressed teleop *intentionally* during an approach to
  cancel the approach — the FSM should yield control.
- (b) Operator already invoked rail_approach with skip_coarse, the
  operator is "the one holding the alignment loop" — the FSM keeps
  rail_approach driving.

The current code cannot tell (a) from (b) because both look like
`operator_mode=="teleop" AND rail_approach_state=="driving"`. The
"who fired rail_approach" information is lost.
**Greenhouse impact**: Operator emergency-cancel of an approach
gone wrong (operator sees the robot drifting toward a plant) takes
two clicks (cancel rail_approach service → set teleop) instead of
one. In a panic scenario, that's a real delay.
**Recommendation**:
1. Add a service `/agv/rail_approach/cancel` (if not present) and
   wire the dashboard's teleop button to call it before changing mode.
2. Alternative: change the carve-out so that an **explicit** operator
   mode transition (set teleop FROM a non-teleop mode) cancels
   rail_approach, while teleop already being active when rail_approach
   fires from outside keeps the carve-out. The FSM would need a
   `previous_operator_mode` bit to distinguish.
3. Document the current behavior in the dashboard so the operator
   knows: "Click ⏹ Cancel before Teleop to stop an active approach."
**Acceptance criterion**: HIL scenario "rail_approach driving →
operator clicks Teleop"; the robot motion goes to zero within 300 ms;
rail_approach_state transitions to `aborted`.
**Effort**: M (FSM bit + service plumbing).
**Prerequisites**: rail_approach exposes a cancel service (verify
separately).

---

#### MEDIUM-11-A-04 — `safety_stop` detection is a substring match, accepts false positives
**File(s)**: `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:174`.
**Category**: bug.
**Symptom**: `msg->data.find("stop") != std::string::npos` matches any
JSON / string payload containing the literal substring "stop" —
including hypothetical values like `"stopped"`, `"no_stop_zone"`,
`"e_stopped"`, `"freestop"`. With CRITICAL-11-A-01 fixed (correct
type), this substring heuristic still bites.
**Analysis**: A `CollisionMonitorState.polygon_name` field carrying
the string `"stop_zone"` (the polygon's NAME, not an action) would
trip `safety_stop`. The current dashboard `collision_monitor.polygon`
field passed via WebSocket
([ws/control.ts:46](../../../src/agv_ui_backend/src/ws/control.ts)) is
`s.collisionMonitor.polygon` — a NAME string. If this same naming
convention flows in the underlying topic, the heuristic mismatches.
**Recommendation**: After CRITICAL-11-A-01 fix, compare
`msg.action_type` to the `STOP` constant from `nav2_msgs`. Drop
substring matching entirely.
**Effort**: NIT (combine with CRITICAL-11-A-01 fix).

---

#### MEDIUM-11-A-05 — Default `operator_mode = "nav"` allows immediate motion on cold boot
**File(s)**: `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp:74` (default), `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:176-180` (subscriber).
**Category**: bug / safety boundary.
**Symptom**: On cold boot, before `/agv/mode/set` is published, the
FSM's `operator_mode` is `"nav"` by struct default. The FSM accepts
Nav2 commands immediately. There is no "wait for operator
confirmation" gate.
**Analysis**: This matters because:
- The boot DAG launches the arbiter at t=7 s
  ([`specs/launch_sequence.yaml#sequence`](../../../specs/launch_sequence.yaml)).
- Nav2 lifecycle activates around t=6 s. By t=7 s, Nav2 is ready to
  accept goals.
- The dashboard backend starts at t=8 s and publishes `/agv/mode` on
  first WS connection (or boot-time event). If the operator's tablet
  was connected and had a pending nav goal queued, that goal could
  fire **before** `/agv/mode/set` arrives — and the arbiter would
  let it through.
- Specs `state_machine.yaml` invariant `mode_coherence` says nav goals
  require Nav2 lifecycle active + motors armed. The arbiter relies on
  upstream gating (`sendNavGoal` in backend). But the arbiter's own
  default of "nav" means it provides zero defense if the upstream
  guard is bypassed (e.g., a ROS-side goal injection).
**Greenhouse impact**: Reboot mid-mission with goal pending → robot
starts moving toward the goal as soon as the arbiter ticks. Tablet
operator may not be ready.
**Benchmark**: Spot SDK requires explicit "power on motors" confirmation
after every boot. AMRs in industrial settings typically have a one-click
"resume" required after boot.
**Recommendation**: Change the FSM default to `operator_mode = "idle"`
([mode_fsm.hpp:74](../../../src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp)). The first dashboard connection (or
explicit operator click) advances to nav/teleop. This is one line.
Update tests if the default leaks into expectations.
**Acceptance criterion**: After a fresh launch, before any
`/agv/mode/set` publish, sending a nav goal does NOT cause motion.
The mode pill shows "idle" until the operator presses Connect / Nav.
**Effort**: S.
**Prerequisites**: none.

---

#### MEDIUM-11-A-06 — Source state strings hardcoded; no schema or enum
**File(s)**: `mode_fsm.hpp:182-211` (rail_driver states), `:226-244` (rail_approach states).
**Category**: bug / brittleness.
**Symptom**: The FSM compares `rail_driver_state` and
`rail_approach_state` against literal strings: `"driving"`, `"reached"`,
`"settled"`, `"aborted"`, `"idle"`, `"blocked_lateral"`,
`"blocked_misaligned"`, `"canceled"`, `"tag_acquisition"`,
`"coarse_approach"`. Any rename or typo in an upstream controller
silently breaks the FSM.
**Analysis**: There is no compile-time link between the strings the
upstream controllers emit and the strings the FSM accepts. The 2-tier
extraction (JSON parse → string compare) is the right pattern for
loose coupling, but the **schema is undeclared**. A controller change
that adds a new state value (e.g., rail_driver gets a `"settling"`
state between "driving" and "reached") would be ignored by the FSM —
behavior unchanged on the surface, regression silently.
**Recommendation**:
1. Define an enum in `agv_interfaces/msg/RailDriverState.msg` and
   `agv_interfaces/msg/RailApproachState.msg` with int8 constants for
   each state. Publish those instead of free-form JSON strings.
2. The FSM gets an explicit enum dependency; compile error on
   unhandled values; tests cover the matrix.
3. Migration: keep JSON publication AND publish the typed message in
   parallel for one release.
**Acceptance criterion**: `git grep '"driving"' src/agv_mode_arbiter`
returns zero hits — all comparisons go through the enum.
**Effort**: M (cross-package refactor).
**Prerequisites**: none, but coordinates with rail_driver and
rail_approach package owners.

---

## 11.B — `agv_waypoint_manager`: mission execution

**Architectural surprise**: the operator dashboard does **not** use
`agv_waypoint_manager` for mission execution. The backend
(`agv_ui_backend/src/index.ts:461`) has its **own** `executeMission()`
that:
- Reads from `${DATA_DIR}/missions/missions.json` (JSON array file).
- Loops over nodes and calls `ros.sendNavGoal` for each.
- Goes through the localization / motors-armed / collision-monitor
  gates ([index.ts:258-300](../../../src/agv_ui_backend/src/index.ts)).

`agv_waypoint_manager_node` runs in the launch but reads a
**different** file (`${nav_dir}/missions/missions.json` — colcon install
share) and is invoked only when a client calls its
`waypoint_manager/execute` service directly. **Nothing in the
production stack appears to call that service** (the dashboard uses
the backend's own executor; no other ROS node consumes it).

Consequences:
- `CR-00-06` (waypoint_manager bypasses localization gate) is **latent,
  not active** for the dashboard flow. The dashboard's mission
  executor DOES go through the gate.
- Two mission executors with two persistent files exist with no
  shared definition. Drift inevitable.
- If a future feature adds a CLI mission launcher or a CI test that
  calls `waypoint_manager/execute`, CR-00-06 becomes live again.

This shifts the priority of `CR-00-06` and adds new findings.

### Findings

---

#### HIGH-11-B-01 — Dashboard uses backend mission executor; `agv_waypoint_manager` is shadow code with separate file
**File(s)**:
- `src/agv_ui_backend/src/index.ts:37` (`MISSIONS_FILE = path.join(DATA_DIR, 'missions', 'missions.json')`)
- `src/agv_ui_backend/src/index.ts:461-545` — mission executor in backend.
- `src/agv_bringup/launch/agv_full.launch.py:75` — waypoint_manager's `missions_file = os.path.join(nav_dir, 'missions', 'missions.json')` (install share).
- `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:73-90` — declares `missions_file` as required parameter.

**Category**: architecture / debt.
**Symptom**: Two mission executors, two different files, two different
formats (backend writes JSON array via `JSON.stringify(missions)`;
waypoint_manager writes/reads line-delimited JSON). Dashboard always
uses the backend's executor; waypoint_manager's services are exposed
but not consumed by any production caller.
**Analysis**: The 2026-04-13 audit `CR-00-06` flagged the bypass
assuming waypoint_manager was the active executor. Verification now
shows the dashboard's `executeMission()` goes through `ros.sendNavGoal`
which **does check** localization / motors / collision_monitor
([index.ts:258-300](../../../src/agv_ui_backend/src/index.ts)). So the
dashboard path is gated.

But this creates two problems:
- **Latent bug**: anyone who invokes `waypoint_manager/execute` (CLI,
  future feature, integration test) gets the un-gated path.
  CR-00-06 still describes a real hazard for that path.
- **Duplicate code**: bug fixes to mission execution have to land
  twice. The two implementations have already drifted (file format
  differs; backend has rail_approach integration at
  [index.ts:486-520](../../../src/agv_ui_backend/src/index.ts) that
  waypoint_manager does not).
- **Pause/resume confusion**: backend has
  `state.missionPause` toggle ([missions.ts:74-83](../../../src/agv_ui_backend/src/routes/missions.ts)).
  Waypoint_manager has no concept of pause. If anyone calls
  waypoint_manager's execute and then POSTs `/api/missions/pause`,
  the pause flag is set but waypoint_manager keeps executing — UI lies.
**Greenhouse impact**: a future operator following CLI docs (`ros2
service call /agv/waypoint_manager/execute ...`) bypasses every
backend gate. CR-00-06 stays a real hazard.
**Recommendation** (two options):
1. **Delete waypoint_manager from production stack** (mark `dev_only:
   true`, remove from `agv_full.launch.py`). Document the dashboard
   executor as the single mission path. Closes CR-00-06 by removing
   the alternative.
2. **Refactor waypoint_manager to route through the backend** — its
   execute service POSTs to the backend's `/api/missions/:id/execute`.
   Keeps the CLI path but funnels it through the gates.
Option 1 is smaller and matches the de facto architecture. Option 2
preserves CLI for development.
**Acceptance criterion**: A ROS service call to
`/agv/waypoint_manager/execute` while localization is `FAILED` is
rejected (option 2) or the service is gone (option 1).
**Effort**: S (option 1) → M (option 2).
**Prerequisites**: none.

---

#### HIGH-11-B-02 — `rclcpp::spin_until_future_complete` called from execution thread on a single-threaded executor
**File(s)**: `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:309-311, 327-329`.
**Category**: bug / concurrency.
**Symptom**: `execute_mission_thread()` runs in a `std::thread`
spawned in `on_execute()`. Inside that thread it calls
`rclcpp::spin_until_future_complete(this->get_node_base_interface(),
goal_handle_future, ...)`. The node itself is spun by
`rclcpp::spin(std::make_shared<WaypointManagerNode>())` in `main()`
([waypoint_manager_node.cpp:368-370](../../../src/agv_waypoint_manager/src/waypoint_manager_node.cpp)).
The default executor is single-threaded. **Two concurrent spin calls
on the same executor are undefined behavior** — typically a deadlock
on a callback group mutex or a recursive-spin assertion.
**Analysis**: The ROS 2 documentation explicitly warns against this
pattern: do not call `spin_until_future_complete` on a node that is
already being spun. Correct alternatives:
- Use a `MultiThreadedExecutor` with a `MutuallyExclusive` callback
  group for the execution thread.
- Use the action's response/result callbacks (registered via
  `SendGoalOptions`) and avoid blocking spin altogether.
- Drop the execution thread and run mission state machine inside a
  timer callback.

That this hasn't surfaced as a hard crash in HIL suggests the executor
internal lock allows recursive entry — but that's an undocumented
side-effect and changing rclcpp versions could break it.
**Greenhouse impact**: Real risk of a rare hang where the mission
thread sits in `spin_until_future_complete` forever because the main
spin has not released the executor lock. The mission's 5-min per-goal
timeout (line 328) would still fire eventually, but the node becomes
unresponsive to other service calls in the meantime.
**Recommendation**: Switch to the action-client callback pattern. Send
goal with a response callback that records the goal handle; record
result via a separate result callback. The mission thread polls a
condition variable instead of spinning the node.
**Acceptance criterion**: Long-running HIL mission with concurrent
`waypoint_manager/list` calls — the list service must respond in
< 100 ms even while a goal is in flight.
**Effort**: M.
**Prerequisites**: none. Compatible with HIGH-11-B-01 fix option 1
(if waypoint_manager is deleted, this finding is moot).

---

#### MEDIUM-11-B-03 — Mission file is append-only, no duplicate-ID enforcement
**File(s)**: `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:162-166` (save), `:179-191` (list).
**Category**: bug.
**Symptom**: `on_save()` opens with `std::ios::app` and writes a new
line. Repeated saves with the same `mission_id` append duplicate
lines. `on_list()` reads every line and returns them all — including
duplicates.
**Analysis**: There is no CRUD enforcement: no update, no delete, no
"replace if exists". A long-lived robot accumulates orphan lines.
`on_execute()` (line 209) takes the **first** match it finds, so the
oldest definition wins regardless of operator intent.
**Recommendation**: Rewrite save to read-modify-write. Use proper
JSON (an array, not JSONL) so the file can be loaded as a single
document. Add a `waypoint_manager/delete` service.
**Effort**: S.

---

#### MEDIUM-11-B-04 — Auto-generated mission IDs use `time_since_epoch().count() % 1e8` — collision within 100 ms
**File(s)**: `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:141-142`.
**Category**: bug.
**Symptom**: `id = "m" + std::to_string(now_ns_count % 100000000)`.
Modulo 1e8 collapses the nanosecond count to a number that rolls every
100 ms. Two saves within 100 ms get the same ID.
**Analysis**: The backend's missions.ts has the same problem with
modulo 1e8 on `Date.now()` ([missions.ts:42](../../../src/agv_ui_backend/src/routes/missions.ts)) but on milliseconds the
rollover is every 27 hours — much less likely. Both should use UUIDs
(`uuid_msgs` / `crypto.randomUUID`) for uniqueness.
**Recommendation**: Use a 128-bit UUID, or at minimum the full 64-bit
nanosecond timestamp without modulo. Cross-fix the backend.
**Effort**: S.

---

#### MEDIUM-11-B-05 — No waypoint validation: NaN, out-of-bounds, in-keepout all accepted
**File(s)**:
- Backend: `src/agv_ui_backend/src/routes/missions.ts:32-55` (POST `/api/missions`).
- Waypoint_manager: `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:134-172` (`on_save`).

**Category**: bug / safety.
**Symptom**: Both mission-creation entry points accept any numeric
`x, y, theta` without validation. NaN, ±inf, coordinates outside the
loaded map's bounds, coordinates inside declared keepout zones — all
get persisted.
**Analysis**: The downstream defenses are imperfect:
- `ros.sendNavGoal` does NOT check coordinates against map bounds or
  keepout zones. Nav2's planner will refuse with "no path found"
  silently — operator sees a generic failure.
- NaN propagates into the quaternion calculation
  (`std::sin(theta/2)`) producing NaN orientations. Nav2 typically
  rejects with a hard fail, but the rejection message is opaque.
**Greenhouse impact**: An operator who fat-fingers a coordinate
(e.g., y=43.7 in a 10×10 m greenhouse) saves a "valid" mission that
will fail at execution time with no clear cause.
**Recommendation**:
1. In `routes/missions.ts` POST handler, validate every waypoint:
   `isFinite(x) && isFinite(y) && isFinite(theta)`, theta in `[-π, π]`,
   and `(x,y)` within the loaded map's bounding box. Reject 400 on
   failure with the offending waypoint index.
2. Keepout-zone check is more complex (need to load `zones.json`); ship
   as a follow-up.
3. Same in waypoint_manager_node's `on_save`.
**Acceptance criterion**: A POST with `{"waypoints":[{"x":NaN,"y":0}]}`
returns HTTP 400 with `{"error": "waypoint 0: x is not finite"}`.
**Effort**: S.

---

## 11.C — `agv_ui_backend`: the TypeScript bridge

The backend has good architectural bones — Express + WebSocket +
rclnodejs cleanly separated, route handlers per concern, an explicit
state machine module. Most findings are in **auth defaults**,
**WebSocket protocol omissions**, and **input validation gaps**, not
in fundamental design.

### Authentication mechanics (verified)

- `src/agv_ui_backend/src/auth.ts:52-173` — `AuthManager` class.
- JWT via `jsonwebtoken` package; secret randomly generated once on
  first boot and persisted to `${AGV_DATA_DIR}/users.json`.
- Password hashing: unsalted SHA-256
  ([auth.ts:48-50](../../../src/agv_ui_backend/src/auth.ts)).
- Roles: `viewer`, `operator`, `engineer` with simple hierarchy
  ([auth.ts:30-34](../../../src/agv_ui_backend/src/auth.ts)).
- `filterActionsForRole` ([auth.ts:37-46](../../../src/agv_ui_backend/src/auth.ts)) returns:
  - engineer → all actions
  - operator → all actions (despite the comment claiming "everything
    except config")
  - viewer → no actions

### Findings

---

#### CRITICAL-11-C-01 — Auth is `enabled: false` by default + hardcoded credentials shipped in source
**File(s)**:
- `src/agv_ui_backend/src/auth.ts:64` — `enabled: false`
- `src/agv_ui_backend/src/auth.ts:67-70` — `engineer / agv2026`, `operator / agv`
- `src/agv_ui_backend/src/auth.ts:80-81` — defaults written to `users.json` on first boot.

**Category**: security / safety.
**Symptom**: Out of the box, the backend accepts unauthenticated
connections. Even when an operator enables auth via
`/api/auth/users PUT enabled=true` later, the default credentials
`operator / agv` and `engineer / agv2026` are now visible in the
public source code. **Any device on the greenhouse LAN can
authenticate as engineer with credentials lifted from this repo.**
**Analysis**: The WS handler enforces:
- `enabled=false` → every connection is granted `'operator'` role
  ([ws/control.ts:75](../../../src/agv_ui_backend/src/ws/control.ts)).
- `enabled=true` → JWT verification; user roles enforced.

But:
- `filterActionsForRole` returns ALL actions for operator role
  ([auth.ts:38-39](../../../src/agv_ui_backend/src/auth.ts)) — the
  comment "operators can do everything except config" is **not
  implemented**. Operator role = engineer role functionally.
- The frontend's `App.tsx:42` swallows errors from
  `getAuthStatus()` and defaults to **logged in**:
  `.catch(() => { setLoggedIn(true) })`. Fail-open.

Combined, the worst-case scenario:
- Repo is public → adversary knows `engineer:agv2026`.
- Operator deploys without changing the credentials → adversary on
  LAN logs in and drives the robot.
- Operator deploys with default `enabled=false` → adversary doesn't
  even need to log in.
**Greenhouse impact**: Connects to **HAZOP H-14** (untrained operator
activates wrong mode). With public credentials the threat surface
includes anyone on the greenhouse WiFi — workers' phones, visiting
maintenance, etc.
**Benchmark**: Industrial AMR controllers (Locus, Spot SDK) refuse to
boot without an admin password set on first run. AWS IoT Core requires
device certificates. The minimum bar for a fleet-attached robot in a
commercial environment is "no shipped credentials".
**Recommendation** (multi-part, ALL required):
1. **Change defaults to `enabled: true`** at line 64.
2. **Remove the hardcoded credentials** from
   ([auth.ts:67-70](../../../src/agv_ui_backend/src/auth.ts)). On
   first boot, generate a **random admin password**, write it to
   `users.json`, and **also log it to stdout** (which goes to
   `journalctl -u agv.service` on the Jetson) with a clear "RECORD
   THIS PASSWORD AT FIRST LOGIN".
3. **Add a forced password change** at first login: the user `admin`
   created above is flagged `must_change_password: true`. The
   `/api/auth/login` flow returns a flag the frontend uses to show
   a "set new password" dialog.
4. **Implement `filterActionsForRole` for `operator`** to actually
   restrict: at minimum, viewers can read, operators can run missions
   and teleop, engineers can additionally edit config / calibration /
   user management.
5. **Fail-closed in `App.tsx:42`**: if `getAuthStatus()` fails, show a
   "Backend unreachable" screen, not a logged-in dashboard.
6. **Switch to a salted password KDF** (bcrypt, argon2id, or scrypt).
   See HIGH-11-C-02 below.
**Acceptance criterion**: A fresh clone + first boot produces a
`users.json` with a randomly-generated `admin` password (logged
once) and `enabled: true`. Attempting `/api/auth/login` with
`engineer / agv2026` returns 401.
**Effort**: M (3–5 h total; multiple files).
**Prerequisites**: none. Belongs to **Sprint A.5** alongside
CRITICAL-11-A-01.

---

#### HIGH-11-C-02 — Unsalted SHA-256 password hashing
**File(s)**: `src/agv_ui_backend/src/auth.ts:48-50`.
**Category**: security.
**Symptom**: `hashPassword(password)` returns
`sha256(password).digest('hex')`. No salt, no key-stretching.
**Analysis**: SHA-256 evaluates in microseconds. A modern GPU computes
> 10⁹ SHA-256/s. Rainbow tables for unsalted SHA-256 of common
passwords are widely available. Two users with the same password get
the same hash, leaking equality.
**Recommendation**: Use `bcrypt` (already wide TypeScript ecosystem),
`argon2id`, or Node's built-in `crypto.scrypt` with per-user random
salt. The `users.json` schema gets a `salt` field.
**Acceptance criterion**: `hashPassword("agv")` produces a different
output every time it is called (because of the salt).
**Effort**: S.
**Prerequisites**: CRITICAL-11-C-01 closes the worst-case threat;
this finding hardens the residual.

---

#### HIGH-11-C-03 — JWT token transported as URL query string in WebSocket connect
**File(s)**: `src/agv_ui_backend/src/ws/control.ts:77-80`, `web/agv_dashboard/src/hooks/useWebSocket.ts:55-57`.
**Category**: security.
**Symptom**: The dashboard appends `?token=<jwt>` to the WebSocket
URL. The server reads `url.searchParams.get('token')`.
**Analysis**: URL query strings are:
- Written to server access logs (Express may log full URL).
- Cached in browser history.
- Visible to any HTTP proxy on the path.
- Logged by CDNs and load balancers.

JWT in the URL is a documented anti-pattern. Correct alternatives:
- `Sec-WebSocket-Protocol` subprotocol header carrying the token.
- HTTP-only cookie set by `/api/auth/login` and validated on WS
  upgrade.
- HTTP basic auth on the WS upgrade request (still not great, but
  better than URL).
**Recommendation**: Move the token to `Sec-WebSocket-Protocol`.
Update the WS server to read the header in the upgrade handler.
Same change in `useWebSocket.ts` — use `new WebSocket(url, [token])`.
**Effort**: S.

---

#### HIGH-11-C-04 — HTTP without TLS — credentials and JWT in cleartext over WiFi
**File(s)**: `src/agv_ui_backend/src/index.ts` (Express setup); README "Build" section; deployment docs.
**Category**: security.
**Symptom**: The backend listens on plain HTTP on port 8090. Operator
credentials (`/api/auth/login`), JWT tokens (all subsequent requests),
and WS frames travel in cleartext over the greenhouse WiFi.
**Analysis**: The threat model is a private greenhouse network, but
"private" is not the same as "trusted". Worker phones on the same
SSID can sniff WPA2-Personal traffic if they have the PSK. WPA3 with
SAE mitigates passive sniffing but the deployment uses WPA2 per
`docs/audit/2026-04-13-full-audit.md` context.
**Recommendation** (two paths):
1. **Self-signed TLS** with a cert pinned in the dashboard. Generated
   on first boot, fingerprint shown in the systemd journal alongside
   the admin password (per CRITICAL-11-C-01). Operator manually
   trusts the cert in the browser on first connect.
2. **Reverse proxy** (caddy / traefik) running on the Jetson that
   terminates TLS with an automatically-renewed self-signed cert.
**Effort**: M.

---

#### HIGH-11-C-05 — `filterActionsForRole` returns all actions for `operator` role despite comment
**File(s)**: `src/agv_ui_backend/src/auth.ts:37-46`.
**Category**: bug / security.
**Symptom**:
```ts
if (role === 'engineer') return actions;
if (role === 'operator') return actions; // operators can do everything except config
```
The comment is aspirational; the code returns the same `actions`
object as for engineer. Operator and engineer are functionally
equivalent.
**Analysis**: HAZOP `H-14` (untrained operator activates wrong mode)
relies on operator-vs-engineer separation. Right now, an operator
account can:
- Add/delete users (`requireAuth('engineer')` gate at
  [routes/auth.ts:27,37](../../../src/agv_ui_backend/src/routes/auth.ts)
  — wait, this IS gated correctly at the route level).
- Read users list (also engineer-gated).
- Everything else (teleop, mission, mode change, e-stop, calibration
  triggers, map operations).

So the *route-level* engineer gates are correctly enforced. What's
broken is the *UI-affordance* signal — `filterActionsForRole`
controls which buttons the dashboard shows. Engineer-only routes
still 403 at the API level. Net: operator sees engineer buttons in
the UI, clicks them, gets 403. UX bug, not a security hole.

But the security hole IS that **the comment says "except config" but
no route distinguishes operator from engineer config-write**, e.g.,
calibration triggers, mode-arbiter parameters. If a future endpoint
adds `requireAuth('operator')` for what *should* be engineer-only,
the boundary is silent.
**Recommendation**:
1. Implement the action-set per role properly. At minimum: viewer
   gets no commanding actions; operator gets teleop / nav / mission;
   engineer gets the operator set + map/calibration/config.
2. Add `requireAuth('engineer')` to every config-write route. Audit
   `routes/*.ts` for "writes that should require engineer".
**Acceptance criterion**: An operator-role JWT cannot call any route
that writes a config file (calibration, AprilTag layout, mode_arbiter
params). The dashboard hides those buttons for operator role.
**Effort**: M.

---

#### MEDIUM-11-C-06 — WebSocket has no application-level heartbeat or deadman
**File(s)**: `src/agv_ui_backend/src/ws/control.ts:144-205` (status broadcast loop), `web/agv_dashboard/src/hooks/useWebSocket.ts` (no pong reply).
**Category**: bug / failure mode.
**Symptom**: The server pushes status at 5 Hz. There is no ping/pong
WebSocket protocol heartbeat to detect dead clients, and no
application-level "operator is alive" signal from client to server.
**Analysis**: TCP keepalive will eventually detect a dead client, but
on Linux defaults that's 7200 s — far too long. The status push at 5 Hz
exposes a `send` error when the socket is broken, which clears the
interval — but **only on next send**. If the client is "half-open"
(packets reaching the server but server replies dropped) the server
keeps sending forever.

More importantly: **HAZOP H-07** ("Loss of WiFi to operator") requires
that an active mission pauses or aborts if the operator disconnects.
There is no such logic. The mission executor's loop
([index.ts:477-534](../../../src/agv_ui_backend/src/index.ts)) checks
`state.missionCancel` and `state.missionPause` but neither flips on
WS disconnect.

`on close` ([ws/control.ts:274-278](../../../src/agv_ui_backend/src/ws/control.ts))
sends `cmd_vel(0,0)` once and decrements active client count. **It
does NOT pause the active mission.** A 12-waypoint mission keeps
running with no operator watching.
**Greenhouse impact**: Operator disconnects (WiFi blip, phone
sleeps, walks away); mission keeps executing unsupervised. Real risk
in a busy greenhouse.
**Recommendation**:
1. Implement WS `ping` from server every 2 s; close connection if
   no `pong` within 5 s.
2. On WS close, if a mission is running and `state.activeClients == 0`,
   set `state.missionPause = true` and emit a `crit` event "Mission
   paused: no operator connected". Resume requires a new client
   connecting AND explicit operator click.
**Acceptance criterion**: HIL scenario "start mission, close WS";
mission_progress.status transitions to `paused` within 5 s. A new
WS connect surfaces the pause to the new client.
**Effort**: M.
**Prerequisites**: none. Closes HAZOP H-07.

---

#### MEDIUM-11-C-07 — `state.missionPause` toggled by REST but ignored by `waypoint_manager_node`
**File(s)**: `src/agv_ui_backend/src/routes/missions.ts:74-83` (pause/resume), `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:286-345` (execution loop only checks `mission_cancel_`).
**Category**: bug.
**Symptom**: The `/api/missions/pause` REST endpoint flips
`state.missionPause`. The backend's own executor reads it
([index.ts:481](../../../src/agv_ui_backend/src/index.ts)) and waits
in a loop. But `agv_waypoint_manager` has no concept of pause. If
anyone calls `waypoint_manager/execute` (cf. HIGH-11-B-01), then
clicks pause on the dashboard, the pause flag flips but the
waypoint_manager mission keeps running.
**Analysis**: Dashboard UX says "Mission paused". Reality says "the
flag is set, the executor doesn't know".
**Recommendation**: Either delete `waypoint_manager` from production
(HIGH-11-B-01 option 1), OR have waypoint_manager subscribe to a
`pause` topic that backend publishes.
**Effort**: S (combined with HIGH-11-B-01).

---

#### MEDIUM-11-C-08 — No input validation on `POST /api/nav/goal`; missing fields silently → (0,0,0)
**File(s)**: `src/agv_ui_backend/src/routes/nav.ts:5-12`.
**Category**: bug.
**Symptom**:
```ts
parseFloat(req.body?.x || 0)
```
If `x` is undefined, `null`, or non-numeric, `parseFloat(0)` returns
0 silently. A goal request with malformed body becomes a goal at
`(0, 0, 0)`.
**Analysis**: `(0, 0, 0)` is typically the map origin — often a
realistic destination (the chargers / docking area in many
greenhouse layouts). The robot would actually try to navigate there.
**Recommendation**: Validate body with a schema (Zod, AJV). Reject
400 on malformed input with a specific error.
**Effort**: S.

---

#### MEDIUM-11-C-09 — No rate limiting on REST endpoints
**File(s)**: `src/agv_ui_backend/src/index.ts` (Express setup), `routes/*.ts` (no middleware).
**Category**: security / robustness.
**Symptom**: A buggy client (or attacker) hitting
`POST /api/missions` 1000×/s fills the missions.json file with
duplicate entries. `POST /api/nav/goal` spam queues a flood of
goals. `POST /api/auth/login` brute-force has no slow-down.
**Recommendation**: Add `express-rate-limit` middleware. Per-route
limits: login 5/min/IP; missions 60/min; nav goals 30/min.
**Effort**: S.

---

## 11.D — `web/agv_dashboard`: the React frontend

Structural audit only. UX usability, accessibility (ARIA, contrast),
and copy quality are **not** assessed — they require a real user study.

### Information architecture (verified)

- `App.tsx` (entry) — auth gate → `Dashboard`.
- `Dashboard` renders: `TopBar` (status + e-stop), `ModeRail` (left
  sidebar: operate / map / missions / recovery / analytics / apriltags
  / waypoint_battery), `MapView` (center), and a context panel per
  ModeRail selection (right).
- Mode (teleop/mapping/nav) visible in `TopBar` and in `OperatePanel`
  active button. Robot state (idle/ready/etc.) derived in
  `state_machine.ts:50-66` and exposed via WS status.
- Robot health surfaces in `RecoveryPanel` (via `status.health`).

### Findings

---

#### HIGH-11-D-01 — Auth check is fail-open in the frontend
**File(s)**: `web/agv_dashboard/src/App.tsx:35-45`.
**Category**: security.
**Symptom**:
```ts
api.getAuthStatus().then(s => { ... }).catch(() => {
  setLoggedIn(true) // if endpoint fails, skip auth
  setAuthChecked(true)
})
```
If `/api/auth/status` fails (backend down, network glitch, CORS
misconfig), the user is **logged in by default**.
**Analysis**: This is fail-open. The motivation was probably "don't
brick the UI if backend is briefly unavailable", but the consequence
is that an attacker who can DDoS just the auth endpoint (while the
WS endpoint stays up) bypasses login.
**Recommendation**: Show a "Backend unreachable" screen instead.
The user explicitly clicks "Retry" to attempt the check again.
**Acceptance criterion**: With `iptables -A INPUT -p tcp --dport
8090 -j DROP` on the Jetson (simulating backend isolation),
opening the dashboard shows a clear error screen, not the
authenticated UI.
**Effort**: S.

---

#### MEDIUM-11-D-02 — E-stop toggle has no protection against accidental disengage
**File(s)**: `web/agv_dashboard/src/components/TopBar.tsx:240-241`.
**Category**: bug / safety UX.
**Symptom**: The TopBar E-stop button is a single-click toggle:
`onClick={() => onEStop(!s?.e_stop)}`. One click engages,
**one click disengages**. An accidental click while E-stop is
engaged (operator passes the tablet to someone, hand brushes the
button) clears the stop.
**Analysis**: E-stop semantics in industrial controllers:
- Engaging E-stop: must be **instant**, no confirmation.
- Clearing E-stop: must be **deliberate** — typically requires a
  separate "Reset" action after the trigger condition is resolved.

The current implementation conflates both into one button.
**Recommendation**: Make the button engage-only; show a separate
"Clear E-Stop" button (or modal with "Acknowledge: condition
resolved?") that appears only when E-stop is engaged. Mirrors
industrial hardware E-stop with a separate reset latch.
**Effort**: S.

---

#### MEDIUM-11-D-03 — No confirmation on Disarm Motors during navigation / mission
**File(s)**: `web/agv_dashboard/src/components/panels/OperatePanel.tsx:34-40`.
**Category**: bug / safety UX.
**Symptom**:
```tsx
<button onClick={() => onMotorEnable(!motorsArmed)}>
  {motorsArmed ? 'Disarm Motors' : 'Arm Motors'}
</button>
```
Single-click disarm. The button is enabled whenever `motorsArmed`,
including during active navigation, mission execution, or rail
approach.
**Analysis**: Disarm during navigation is a hard stop — motors enter
IDLE state, robot coasts to a stop. Not safety-dangerous (robot
stops) but operationally disruptive: an in-flight mission aborts;
the operator must arm, re-localize, and resume.
**Recommendation**: When `motorsArmed && (state == 'navigating' ||
state == 'executing_mission')`, show a modal: "Disarming will halt
the active mission. Continue?".
**Effort**: S.

---

#### MEDIUM-11-D-04 — `handleGoalClick` sends nav_goal on single map click; no confirmation
**File(s)**: `web/agv_dashboard/src/App.tsx:111-117`.
**Category**: bug / UX.
**Symptom**: In nav mode, clicking anywhere on the map sends
`nav_goal` with `theta=0` immediately. A mis-click sends the robot
elsewhere.
**Analysis**: At 0.25 m/s vx_max, a 10 m mis-clicked goal takes
~40 s to execute. Operator can cancel. But the muscle-memory click-to-
explore behavior of map widgets means an explorative click sends a
real command.
**Recommendation**: Two-stage interaction: first click drops a
ghost waypoint; "Confirm" button sends. Or click+drag with a
visible vector → release sends. Same gesture model as Google Maps
"directions to here".
**Effort**: M (UI redesign).

---

#### MEDIUM-11-D-05 — No calibration wizards
**File(s)**: absence — no calibration panel in
`web/agv_dashboard/src/components/panels/`; no calibration routes in
`src/agv_ui_backend/src/routes/`.
**Category**: missing feature.
**Symptom**: The four calibration procedures expected by the prompt
(camera intrinsics, camera-IMU extrinsics, wheel UMBmark, hand-eye)
are not exposed in the dashboard. Operator must run them from a
terminal.
**Analysis**: The repo has `tools/calib_umbmark.py`,
`tools/calib_motor_ff_*.py`, etc. (per Phase 0 inventory). These are
CLI scripts. The acceptance criterion "<2 h commissioning without
terminal" from the Phase 11 prompt is unmet structurally — there are
no wizards.

The `docs/calibration/odrive_nvram_dump_procedure.md` produced in
Sprint A (commit `61a973c`) explicitly directs the operator to use
`odrivetool` in a terminal. Sprint A scope (no-hardware) did not
include wizards.
**Recommendation**: Plan calibration UI as Sprint E. For each of the
4 wizards: a new panel with step-by-step instructions, capture button
that triggers the CLI script via a backend endpoint, progress
indicator, and validation pass/fail with thresholds (e.g., UMBmark
residual error < 5 %).
**Effort**: XL (4 wizards × M each, plus backend route work).
**Prerequisites**: stable calibration scripts (already exist).

---

#### MEDIUM-11-D-06 — No "stuck in recovery" surface to operator
**File(s)**: `web/agv_dashboard/src/components/panels/RecoveryPanel.tsx` (not read in detail), Phase 7 `MEDIUM-07-07` (BT timeout absence).
**Category**: bug / UX.
**Symptom**: When Nav2's BT enters the recovery RoundRobin (per
`navigate_to_pose_forward_only.xml`) the operator sees `nav_state.active=true` and
`status="active"` in the dashboard. No indication that the robot is
cycling through recoveries instead of making progress.
**Analysis**: The recovery cycle can run for ~60 s before the BT
aborts (Phase 7 `MEDIUM-07-07`). During that minute, the operator
sees a normal "navigating" state. The behavior_tree's recovery
actions (`spin`, `clear_costmap`, `wait`) ARE published as `info`
events to the event log, but they get drowned by other events.
**Recommendation**: Add a derived state in `state_machine.ts`:
`recovering` (set when recovery action emits + `distance_remaining`
unchanged for N seconds). Surface as a yellow pill in TopBar with
"Recovery: rotating 90°" detail.
**Effort**: M.
**Prerequisites**: MEDIUM-07-07 (BT global timeout).

---

#### LOW-11-D-07 — Token stored in localStorage; XSS-exfiltratable
**File(s)**: `web/agv_dashboard/src/api/client.ts` (`api.setToken`, `api.getToken`); `App.tsx:54` uses `api.setToken(null)` on logout.
**Category**: security.
**Symptom**: JWT stored in `localStorage` is accessible to any
JavaScript running on the page. An XSS injection could exfiltrate
the token.
**Analysis**: The frontend has a small attack surface (React, no
`dangerouslySetInnerHTML`, no eval of user input visible in App.tsx).
Risk is low but standard practice is HTTP-only cookies.
**Recommendation**: Use HTTP-only cookies set by the login endpoint.
Combine with HIGH-11-C-03 (move token off WS URL).
**Effort**: M.

---

#### LOW-11-D-08 — No max reconnect cap on WebSocket; infinite retry hides backend down
**File(s)**: `web/agv_dashboard/src/hooks/useWebSocket.ts:46-82`.
**Category**: bug / UX.
**Symptom**: Reconnect uses exponential backoff (500 ms → 5000 ms)
forever. If the backend is permanently down, the dashboard shows
"connecting..." forever with no escalation.
**Recommendation**: After N failed reconnect attempts (e.g., 30,
~3 min total at 5 s cap), show a "Backend offline — robot may be
powered down. Last known status: <stale>" banner with a manual
"Retry" button.
**Effort**: S.

---

## 11.E — Commissioning walkthrough

The end-to-end "non-ROS technician → first successful mission" flow
is documented in a separate file: see
[`11_commissioning_walkthrough.md`](11_commissioning_walkthrough.md).
That document does not file individual findings; it identifies the
**structural breaks** in the commissioning flow where today the
operator must use a terminal, no validation exists, or no wizard
is present.

---

## Re-rating of prior findings

| Finding | Prior | Now | Rationale |
|---|---|---|---|
| `CR-00-06` (waypoint_manager bypasses gates) | HIGH | **HIGH (latent)** | Backend executor doesn't use waypoint_manager today, so the operational risk is lower. But the bypass is real if anyone invokes waypoint_manager's service directly. HIGH-11-B-01 proposes deletion as the canonical fix. |
| `H-14` (untrained operator) | R=9 | **R=15** | With CRITICAL-11-C-01 (default credentials + auth disabled), P rises from 3 to 5. Recommend Sprint A.5 closes the credential issue, which lowers P back to 2 (mature auth flow). |
| `H-07` (loss of WiFi to operator) | R=12 | **R=12 (confirmed open)** | MEDIUM-11-C-06 confirms no deadman / heartbeat. Operator disconnect leaves mission running. Mitigation tracked in Sprint B. |
| `MEDIUM-07-07` (no global mission timeout in BT) | MEDIUM | **MEDIUM** (related: MEDIUM-11-D-06 surfaces recovery state) | Same severity; the new finding adds a UX layer. |

---

## Status

| Sub-section | Findings | Severity mix |
|---|---|---|
| 11.A mode_arbiter | 6 | 1 CRITICAL, 2 HIGH, 3 MEDIUM |
| 11.B waypoint_manager | 5 | 2 HIGH, 3 MEDIUM |
| 11.C ui_backend | 8 | 1 CRITICAL, 4 HIGH, 3 MEDIUM |
| 11.D web/agv_dashboard | 8 | 1 HIGH, 5 MEDIUM, 2 LOW |
| 11.E walkthrough | n/a | See `11_commissioning_walkthrough.md` |
| **Total** | **27** | **2 CRITICAL / 9 HIGH / 14 MEDIUM / 2 LOW** |

Within the prompt's expected band (17–29). Two findings rise to
**CRITICAL** and trigger a new **Sprint A.5** (see SUMMARY.md).

End of Phase 11.
