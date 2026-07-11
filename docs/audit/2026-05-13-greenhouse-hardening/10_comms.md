# Phase 10 — Communications and Diagnostics

> Phase 10 goal (per prompt): "Sistema observable en producción, con
> telemetría mínima viable y diagnósticos accionables."

Static audit of DDS configuration, QoS profiles across the workspace,
`/diagnostics` coverage, and rosbag2 policy.

---

## A. QoS profile audit

Survey of `rclcpp::QoS(...)` and `SensorDataQoS` calls across `src/agv_*`:

| Pkg | Topic | QoS | Verdict |
|---|---|---|---|
| `agv_safety` | `~/status` (SafetyStatus) | `QoS(10).reliable()` | ✅ |
| `agv_safety` | `software_estop` sub | `QoS(10).reliable().transient_local()` | ✅ latched |
| `agv_safety` | (`monitored_topics`, type-erased) | `QoS(10).best_effort()` | ⚠️ see `HIGH-09-02` |
| `agv_safety/cmd_vel_gate` | `cmd_vel_out`, `cmd_vel_in`, `safety_status`, `hardware_estop` | reliable; hardware_estop is `transient_local` | ✅ |
| `agv_map_manager` | `maps/loaded`, `current_map` (pub+sub) | `QoS(1).transient_local().reliable()` | ✅ latched state |
| `agv_factor_graph` | `odometry/global`, `/visual_slam/tracking/odometry` (subs) | `SensorDataQoS()` (best_effort, depth 5) | ✅ matches sensors |
| `agv_factor_graph` | `marker_pose` (sub) | `QoS(10)` (default reliable) | ✅ matches publisher |
| `agv_localization_init` | `localization/state` (pub) | `QoS(1).transient_local().reliable()` | ✅ latched |
| `agv_localization_init` | `maps/loaded` (sub) | `QoS(1).transient_local().reliable()` | ✅ |
| `agv_localization_init` | `marker_pose`, `mode` (sub) | `QoS(10)` (default reliable) | ✅ |
| `agv_localization_init` | `/agv/zed/pose_with_covariance`, `/agv/zed/pose/status` (sub) | `SensorDataQoS()` | ✅ |
| `agv_markers` | `marker_pose` (pub), `marker_detected`, `marker_raw_detected` | `QoS(10)` (default reliable) | ✅ |
| `agv_markers` | `odometry/global` (sub) | `SensorDataQoS()` | ✅ |
| `agv_markers` | `markers/registry_reload` (sub) | `QoS(1).transient_local().reliable()` | ✅ latched |
| `agv_rail_approach` | `localization/state` (sub) | `QoS(1).reliable()` | ⚠️ **see `MEDIUM-10-06` below** |
| `agv_rail_driver` | state publisher | `QoS(1).reliable()` (depth=1, reliable) | ✅ — comment line 95 explains the depth=1 choice |
| `agv_image_server` | image subs | `SensorDataQoS()` | ✅ |
| `agv_sensor_fusion/imu_filter` | (not read this audit) | per CLAUDE.md `best_effort` | ✅ |
| `agv_sensor_fusion/covariance_override` | wheel/visual/imu subs | `SensorDataQoS()` | ✅ |
| `agv_sensor_fusion/caster_dwell_advisor` | cmd_vel sub | `QoS(KeepLast(10)).best_effort()` | ✅ |
| `agv_sensor_fusion/fusion_monitor` | `/diagnostics` (pub) | `QoS(10)` (default reliable) | ⚠️ depth 10 may overflow at 1 Hz vs 10 Hz sub — minor |

**Overall verdict**: the QoS discipline in this workspace is **good**.
Pattern is consistent: sensor data uses `SensorDataQoS()` (best_effort,
depth 5), commands/state use reliable+depth 10, latched state uses
`transient_local`+reliable+depth 1. The only systemic concern is the
type-erased subscription in `safety_supervisor` (already filed as
`HIGH-09-02` in Phase 9).

### Findings

---

#### MEDIUM-10-06 — `agv_rail_approach` subscribes to `localization/state` RELIABLE but the publisher is `transient_local.reliable.depth=1`; mismatch tolerated by DDS but the late-join semantics differ from the producer
**File(s)**:
- `src/agv_rail_approach/src/rail_approach_node.cpp:162` — sub uses
  `QoS(1).reliable()` (no `transient_local`).
- `src/agv_localization_init/src/auto_init_orchestrator_node.cpp:166-168` —
  pub uses `QoS(1).transient_local().reliable()`.

**Category**: bug / robustness.
**Symptom**: A non-transient_local subscriber to a transient_local
publisher will **not receive the latched last message** on first
connect; it only receives messages published after the subscription is
established. If `rail_approach` starts after `auto_init_orchestrator`
has already declared LOCALIZED, `rail_approach` doesn't know — it
will wait for the next state transition (which may not come for
minutes).
**Analysis**: All other consumers of `localization/state` use
`transient_local` (the dashboard backend, marker_correction's reload
flow). `rail_approach` is the exception and would silently miss the
state. DDS allows this (the durability QoS request is "best available"
from the producer side), so no warning fires.
**Greenhouse impact**: After a robot reboot, if `rail_approach` is
asked to dock before any localization state transition (which is
realistic: stack boots in LOCALIZED quickly via ZED Area Memory,
operator clicks "dock" immediately), `rail_approach` doesn't know
it's localized and may refuse to start.
**Recommendation**: Change `rail_approach_node.cpp:162` to
`QoS(1).transient_local().reliable()` to match the publisher.
**Acceptance criterion**: A scripted reboot followed immediately by
a dock command succeeds without operator-visible delay.
**Effort**: S (one-line change).
**Prerequisites**: none.

---

## B. DDS / CycloneDDS configuration

The boot script generates `/tmp/agv_cyclonedds_runtime.xml` from a
whitelist of interfaces. See `agv_start.sh:90-155`.

### B.1 Strengths

- **Dynamic generation** at boot avoids the "interface listed but not
  up → SIGABRT" trap.
- **Operstate + carrier check** (line 96–100): both must be `up`/`1`
  before an interface is included.
- **Fallback to localhost-only** (line 121–124) when no whitelisted
  iface is up — graceful degradation.
- **Buffer sizes raised** (line 144–147): `SocketReceiveBufferSize 64 MB`,
  `WhcHigh 4 MB` — sized for ZED RGB HD bursts. Comment explains the
  iteration history.
- **Multicast enabled** when interfaces are present (line 117), SPDP-only
  fallback otherwise (line 124).

### B.2 Gaps already filed in Phase 0

- **CR-00-04**: hardcoded `eno1 wlP1p1s0` interface names (AGX Orin
  DevKit-specific).

### Additional findings

---

#### MEDIUM-10-03 — `MaxAutoParticipantIndex 120` is high for single-robot deployment
**File(s)**: `src/agv_bringup/scripts/agv_start.sh:131`.
**Category**: NIT / performance.
**Symptom**: `<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>`
allocates 120 participant slots. A single-robot stack has on the order
of 25 ROS nodes (one DDS participant per node). 120 is ~5× headroom.
**Analysis**: Each participant slot consumes a small amount of DDS
state on the network discovery side. 120 is fine for a single robot;
for a fleet of robots on the same `ROS_DOMAIN_ID`, this needs to
accommodate every node × every robot. The value would need to grow.
**Greenhouse impact**: None today. Future fleet deployment is the
trigger.
**Recommendation**: Either leave alone with a comment "sized for future
fleet", or shrink to 30 with a note "raise when fleet > 2 robots".
**Effort**: NIT.

---

#### LOW-10-05 — `SocketReceiveBufferSize 64 MB` may exceed Orin NX `net.core.rmem_max` default
**File(s)**: `src/agv_bringup/scripts/agv_start.sh:144`.
**Category**: docs / config.
**Symptom**: The kernel limits per-socket receive buffer via
`net.core.rmem_max` (default ~200 KB on stock Ubuntu, ~8 MB on stock
JetPack). Requesting 64 MB without raising the sysctl produces a silent
clamp — Cyclone gets whatever the kernel allows.
**Analysis**: On the AGX Orin 64 GB devkit the team configured this is
likely not the limiting factor (they've measured the audit-fix is real).
On the Orin NX 16 GB production target the JetPack defaults may differ.
**Recommendation**:
1. Document required sysctl in `docs/hardware_setup.md`:
   ```
   net.core.rmem_max = 67108864
   net.core.wmem_max = 67108864
   ```
2. `agv_start.sh` should `sysctl -n net.core.rmem_max` and warn if
   below 64 MB (the requested cyclone value).
3. Provide a `/etc/sysctl.d/99-agv.conf` to ship in the systemd
   deployment.
**Acceptance criterion**: A fresh Orin NX 16 GB flash with the
documented sysctl achieves the configured CycloneDDS buffer size, and
agv_start.sh logs a clear warning if not.
**Effort**: NIT.

---

## C. `/diagnostics` coverage

Grep reveals **exactly one** publisher: `fusion_monitor_node` at
`src/agv_sensor_fusion/src/fusion_monitor_node.cpp:168`.

Field-side: `src/agv_bringup/scripts/field_test.py:98` is a **subscriber**
(commissioning tool). No `diagnostic_aggregator` is launched in
`agv_full.launch.py`.

### Coverage gaps

| Subsystem | Has `/diagnostics` publisher | Should have |
|---|---|---|
| `fusion_monitor` (localization health) | ✅ yes | yes |
| `agv_odrive` (motor temps, VBUS, errors) | **no** | yes — temps and errors are published as `/agv/motor_state` (JSON string) but not `/diagnostics` |
| `agv_safety` (safety_supervisor verdict) | **no** | yes — `SafetyStatus` is custom message, not DiagnosticArray |
| ZED camera + cuVSLAM | (external; unclear from this repo) | yes if not |
| Nav2 stack | (Nav2 internally publishes per-server) | yes — already present from upstream |
| `/agv/scan` source (pointcloud_to_laserscan) | likely no | yes |

### Findings

---

#### MEDIUM-10-01 — No `diagnostic_aggregator` configured; per-node `/diagnostics` patchy
**File(s)**: absence in `agv_full.launch.py`; only `fusion_monitor`
publishes.
**Category**: bug / observability.
**Symptom**: Only one node in the workspace publishes `/diagnostics`.
There is no `diagnostic_aggregator` to combine, prioritise, and surface
to the operator. The dashboard's "system health panel" (per
`agv_localization_init/CLAUDE.md` improvement opportunity) cannot
currently aggregate per-subsystem status because the data isn't there.
**Analysis**: The team has chosen to use **custom topics** per
subsystem (`SafetyStatus`, `motor_state` JSON, `localization/state` JSON,
`zone/state` JSON, etc.) instead of the ROS-standard `/diagnostics`
aggregation. Pros: typed messages, clean APIs. Cons: every
subscriber has to know about every custom topic; no off-the-shelf tool
(`rqt_robot_monitor`, `runtime_monitor`) works; the dashboard has to
reimplement aggregation.
**Greenhouse impact**: Operator visibility is patched together across
many topics. A new operator faces a fragmented "what is wrong with the
robot" experience. The dashboard does aggregate, but it's
custom — fragile to new subsystem additions.
**Recommendation** (3 options):
1. **Stay custom**: document the custom-topic pattern in
   `specs/interfaces.yaml` as the official approach, with the dashboard
   as the single aggregator. Drop `/diagnostics` entirely. Honest about
   the architectural choice.
2. **Add a diagnostic_updater wrapper** to every critical node:
   `agv_odrive`, `agv_safety`, scan pipeline, `auto_init_orchestrator`.
   Run `diagnostic_aggregator` with a `analyzers.yaml`. Dashboard
   consumes `/diagnostics_agg` instead of N custom topics.
3. **Both**: keep custom topics for HMI-specific structured payloads,
   and **also** publish a coarse `/diagnostics` summary per node for
   external observability tools (Foxglove, rqt_robot_monitor).
**Acceptance criterion**: Either a documented ADR explicitly choosing
"no `/diagnostics`" with rationale, or every critical-path node
publishes `/diagnostics` and the aggregator runs.
**Effort**: S (ADR option) → L (full aggregation rollout).
**Prerequisites**: none.

---

## D. rosbag2 policy

`grep` for `rosbag` / `mcap` in `src/` and `scripts/` returns only
`field_test.py:10,51` (a commissioning script that records a fixed
topic list to a fixed path). There is **no rosbag2 in the production
launch**.

This is a **deliberate choice** — production deployment can't afford
to write 8 h × 5 day continuous bag (the prompt's endurance target).
But the absence is total: there is **no rotation policy, no
event-triggered recording, no MCAP backend, no upload selection**.

### Findings

---

#### MEDIUM-10-04 — No rosbag2 rotation or event-triggered recording in production launch
**File(s)**: absence — no rosbag2 in `agv_full.launch.py`. Only
`field_test.py` records, and only when manually invoked.
**Category**: bug / observability.
**Symptom**: When something goes wrong in the field, the only post-hoc
record is `journalctl -u agv.service` (text logs of the launch) + the
custom event log in `~/agv_data/events.jsonl`. There is no
time-synchronised rosbag of topics that would let an engineer replay
the failure offline.
**Analysis**: Continuous recording is impractical (disk and CPU cost).
But a **rolling buffer** of N seconds, dumped on event trigger, is the
industry standard:
- Record continuously to a `mcap` ring buffer of, say, 30 s.
- On `/agv/safety/status.safety_ok == false` or `nav goal failed`,
  flush the ring + the next N seconds to disk.
- Tag with timestamp + event type; upload selectively when WiFi is
  good.
**Greenhouse impact**: Field failures cannot be diagnosed remotely.
Engineer has to drive out, replicate, and observe.
**Benchmark**: Foxglove Studio recommends `mcap` with `--max-bag-duration`
+ `--max-bag-size`. Autoware ships an "event-triggered logging" launch
file. Tier IV has open-sourced `event_recorder` for this exact pattern.
**Recommendation**:
1. Add a `rosbag2` recorder node to `agv_full.launch.py` with:
   - MCAP backend.
   - Ring buffer 30 s of safety-critical topics
     (cmd_vel chain, scan, odometry/global, marker_pose,
     localization/state, safety/status).
   - Trigger via service call or subscription to event-trigger topic.
2. Add a dashboard button "Dump last 30 s for debugging" that calls the
   trigger.
3. Set rotation to keep 7 days of triggered dumps in
   `${AGV_DATA_DIR}/bags/`.
**Acceptance criterion**: A field event (collision, stuck, loc-fail)
results in a 30 s mcap dump in `${AGV_DATA_DIR}/bags/` viewable in
Foxglove with full topic content.
**Effort**: M.
**Prerequisites**: `${AGV_DATA_DIR}` portability (CR-00-03 closed).

---

#### MEDIUM-10-02 — `safety_supervisor` BEST_EFFORT subscriber to potentially RELIABLE publishers (cross-reference, NIT)
**File(s)**: `src/agv_safety/src/safety_supervisor.cpp:119`.
**Category**: cross-reference of `HIGH-09-02`.
**Symptom**: Already filed in Phase 9. Mentioned in this Phase 10 only
because it is a QoS-discipline issue at heart.
**Recommendation**: See `HIGH-09-02`.
**Effort**: see HIGH-09-02.

---

## E. Diagnostic-relevant items already filed in other phases

- `HIGH-04-02` — incidence-based AprilTag covariance scaling would also
  surface as an EKF diagnostic.
- `MEDIUM-01-04` — clock-skew warning should reach `/diagnostics`.
- `MEDIUM-09-03` — promoting `safety_supervisor` to lifecycle node
  would integrate it into the lifecycle_manager's reporting.

---

## F. Hardware-dependent items

| Acceptance criterion (per prompt §10) | Harness |
|---|---|
| All critical nodes publish to /diagnostics with semantic states | After `MEDIUM-10-01` close, automatic |
| 8 h session generates < 20 GB bag with rotation | After `MEDIUM-10-04` close, measure on hardware |
| Recovery of 5 min WiFi loss: no mission interruption, no critical data loss | Field test |

---

## G. Status

| Item | Status |
|---|---|
| QoS profile audit | ✅ filed `MEDIUM-10-06` |
| DDS config logic | ✅ filed `MEDIUM-10-03`, `LOW-10-05` |
| `/diagnostics` coverage | ✅ filed `MEDIUM-10-01` |
| rosbag2 policy | ✅ filed `MEDIUM-10-04` |
| Cross-reference safety QoS | ✅ `MEDIUM-10-02` notes `HIGH-09-02` |
| Hardware-dependent items | ⏸ deferred (§F) |

End of Phase 10. 4 new findings + 2 cross-references. 1 MEDIUM, 1 MEDIUM, 1 MEDIUM, 1 MEDIUM (10-01, 10-03, 10-04, 10-06) + 1 LOW (10-05).
