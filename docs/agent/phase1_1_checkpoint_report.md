# Sub-fase 1.1 — early checkpoint report

**Date:** 2026-05-13
**Branch:** `claude/amr-security-audit-gPtCd`
**Operator:** Andrés Islas
**Agent:** Claude Code (Opus 4.7, 1M context)

This report covers ~75% of the Sub-fase 1.1 spec. The Sub-fase
1.1.c (System Health Panel UI) work is NOT in this commit set —
explanation in §5.

---

## 1. Status per work unit (verdicts per prompt §0)

| Work | Status | Where |
|---|---|---|
| 1.1.a `verify_topic_types.py` | **CLOSED-VERIFIED-HW** | commit `b051147` + docs in `b264054` |
| 1.1.b backend ROS-independent | **CLOSED-VERIFIED-CODE** (partial) | commit `12f2771` |
| 1.1.c System Health Panel UI | **NOT-STARTED** | scope decision below |

Verifier baseline: `bash tools/verify_specs/all.sh` → 12 scripts,
0 blocking, **0 warnings** (first clean run of the cycle).

---

## 2. 1.1.a verify_topic_types.py — full closure

Already documented in `docs/agent/phase1_log.md` §"Sub-fase 1.1.a".
Summary of the three tests:

1. **Positive** (current workspace): scanned 134 declarations,
   65 cross-checked against `specs/interfaces.yaml`, 0 mismatches.
2. **Negative** (artificial regression): reverted `rail_driver_node.cpp:96`
   to `std_msgs::msg::String` (uncommitted) → verifier exits 1
   BLOCKING with the exact file:line and prior-occurrences history
   in the error message.
3. **Histórico** (against git history): swapped in pre-fix file
   content from commit `08ac348~1` (rail_driver) and `8d81517~1`
   (mode_arbiter / original CRITICAL-11-A-01) → verifier detects
   each bug at the exact pre-fix line. Concrete evidence the
   verifier would have prevented both prior occurrences from
   shipping.

The check is wired into `tools/verify_specs/all.sh` as BLOCKING. A
fourth occurrence of this bug class cannot ship past the gate.

---

## 3. 1.1.b backend RosBridgeProxy + status API — partial closure

Honest scope: the prompt spec §3.2 asks the HTTP/WS server to start
BEFORE `rclnodejs.init()`. The full implementation requires
extracting ~870 lines of intertwined publisher/subscriber/state
setup from `main()` into a `buildRosImpl(node, deps)` function plus
relocating `setMode`/`executeMission`. That refactor is bounded but
substantial (~1 day of focused effort). It was **DEFERRED**.

What landed (commit `12f2771`):

- `src/agv_ui_backend/src/ros_lifecycle.ts` — `RosBridgeProxy` class
  with stable reference + status state machine + listener API. Plus
  a `RosLifecycleManager` skeleton (retry loop, health-check
  placeholder) NOT yet wired.
- `src/agv_ui_backend/src/routes/system.ts` — `GET /api/system/ros_status`
  endpoint reading from the proxy.
- `src/agv_ui_backend/src/index.ts` — `const ros` renamed to
  `const realRos`; `deps.ros = rosProxy`; `rosProxy.setImpl(realRos)`
  after `rclnodejs.spin(node)`.

Verification:
- npm build clean.
- Service restart successful.
- `curl /api/system/ros_status` returns `{"status":"online","detail":"ROS bridge active"}`.

What does NOT work:
- Cold-boot without ROS (spec §3.4 test #1): if `rclnodejs.init()`
  throws/hangs, `server.listen()` still never reached. The
  operator's trauma scenario is NOT closed.

The proxy contract is in place. When the server-first refactor
lands as a separate commit, no panel changes are required — the
panel reads the same `/api/system/ros_status` and observes the
proxy's state regardless of how the underlying impl is wired.

Tracked in `docs/agent/future_work.md` under §"Server-first
bootstrap" with explicit test plan.

---

## 4. ODrive G5 + ekf HIGH-04-09 — earlier-session work shipped

Outside the strict Sub-fase 1.1 spec but landed in this session
because the operator authorized them after Section 0:

- **G5** — ODrive bilateral asymmetry firmware-write
  (commit `2a53bb8`). LEFT vel_integrator_gain 0.333→0.167, RIGHT
  can.encoder_msg_rate_ms 0→10. NVRAM persistence verified post-reboot.
- **HIGH-04-09** — ekf_local `imu0_config` orientation row [T,T,T]
  → [F,F,F] (commit `0428b39`). `CLOSED-VERIFIED-CODE`. Empirical
  re-validation of the +2.4°→<0.5° yaw-jump improvement is
  deferred to the first hardware USB-reboot test during Phase 1
  operations.

---

## 5. 1.1.c System Health Panel UI — NOT started, scope decision

The full 1.1.c spec calls for ~1.5 days of focused work:
- `health_monitor.yaml` config with ~13 components.
- 5 new REST endpoints (`/api/health/components`, .../:id, .../restart,
  .../logs, .../verifiers, .../verifiers/:name/run).
- WebSocket `health_update` events.
- React panel in TopBar with collapsible sections.
- JSONL event persistence with daily rotation, 7-day retention.
- 6 empirical tests on hardware (boot normal, ROS down, sensor
  disconnect, sensor reconnect, verifier from UI, restart from UI).

This session's context budget is finite. Forcing the panel through
in one continuous session risks shipping rushed UI code on top of
the LEAN 1.1.b — when the panel needs the full server-first
bootstrap to actually solve the operator's stated trauma scenario
(panel works → ROS goes down → panel still works → operator
diagnoses).

**Recommendation**: split 1.1.c into a focused next session, AFTER
the server-first refactor (1.1.b's `DEFERRED` portion) lands.
Order:
1. Server-first refactor (~1 day): extract `buildRosImpl`, reorder
   `main()`, add periodic health check, add WS broadcast. Tests
   1–4 of spec §3.4.
2. 1.1.c panel UI (~1.5 days): consume the now-fully-functional
   `/api/system/ros_status` + new `/api/health/*` endpoints. Tests
   1–6 of spec §4.6.

This is honest about scope. The infrastructure groundwork (1.1.a
verifier + 1.1.b proxy + endpoint) is in place; the panel work
benefits from a stable foundation rather than rushing on top of a
partial bootstrap.

---

## 6. Files touched this session

| Path | Action | Sub-fase |
|---|---|---|
| `tools/verify_specs/verify_topic_types.py` | created (commit b051147) | 1.1.a |
| `tools/verify_specs/all.sh` | added BLOCKING entry | 1.1.a |
| `tools/verify_specs/README.md` | created | 1.1.a |
| `docs/agent/phase1_log.md` | created | 1.1.a + 1.1.b |
| `docs/agent/future_work.md` | created | 1.1.a + 1.1.b |
| `src/agv_ui_backend/src/ros_lifecycle.ts` | created | 1.1.b |
| `src/agv_ui_backend/src/routes/system.ts` | created | 1.1.b |
| `src/agv_ui_backend/src/routes/index.ts` | registered new routes | 1.1.b |
| `src/agv_ui_backend/src/index.ts` | proxy wiring + rename | 1.1.b |
| `src/agv_sensor_fusion/config/ekf_local.yaml` | HIGH-04-09 fix | side closure |
| `docs/calibration/odrive_g5_writes_2026-05-13.txt` | created | G5 |

Plus all prior Section-0 and Sprint-X commits from earlier in
session: `0428b39`, `2a53bb8`, `e6804e8`, `976a229`, `2d4c972`,
`d3100f7`, `08ac348`, `f0aac76`.

---

## 7. Time vs estimate

Prompt estimate: 3 days, escalate at 4.

Actual session time: substantial portion of a single continuous
session. Cause of overrun:
- Section-0 Day 1 + Day 2 + SSOT-revert sprint absorbed earlier
  context (work done before this Sub-fase 1.1 prompt landed).
- The 1.1.b full refactor scope (~870 lines of intertwined `main()`
  body) was larger than expected and would have left the system in
  a half-refactored state if pushed in this session.

Pragmatic call: ship the proxy + endpoint contract solidly,
checkpoint, and let the operator decide the order for the
remaining server-first refactor and the panel UI.

---

## 8. Recommendations for Sub-fase 1.2 (and 1.1 remainder)

### Immediate next sprint (proposed)

**Sub-fase 1.1.b.full** (~1 day): the server-first refactor. Tests
in spec §3.4 become the acceptance criteria. After this lands, the
operator's trauma scenario is empirically closed.

**Sub-fase 1.1.c** (~1.5 days): the full System Health Panel UI on
top of a stable backend.

### Stop and wait — per prompt §5.4

This report constitutes the §5.3 deliverable. Awaiting operator
OK before proceeding.
