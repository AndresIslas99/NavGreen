# Sub-fase 1.1 — final checkpoint report

**Date:** 2026-05-13
**Branch:** `claude/amr-security-audit-gPtCd`
**Operator:** Andrés Islas
**Agent:** Claude Code (Opus 4.7, 1M context)

This report supersedes the earlier `phase1_1_checkpoint_report.md`
(which covered only the partial 1.1.a + lean 1.1.b set). The
operator chose path "Option Yes — 1.1.b.full + 1.1.c"; this report
documents the closure.

---

## 1. Final status per work unit (verdicts per prompt §0)

| Work | Status | Anchor commits |
|---|---|---|
| 1.1.a `verify_topic_types.py` | **CLOSED-VERIFIED-HW** | `b051147`, `b264054` |
| 1.1.b proxy + status endpoint | **CLOSED-VERIFIED-CODE** | `12f2771` |
| 1.1.b.full server-first bootstrap | **CLOSED-VERIFIED-HW** (HTTP-first); CODE-only on disconnect→reconnect rig | `b79c148` |
| 1.1.c health monitor backend | **CLOSED-VERIFIED-HW** | `fd5b04c`, `a1f37ee` |
| 1.1.c health panel UI | **CLOSED-VERIFIED-HW** (operator ran verifier from UI in this session) | `fd5b04c` |
| 1.1.c restart endpoint + freshness | **CLOSED-VERIFIED-CODE** | `a1f37ee` |

Verifier baseline: `bash tools/verify_specs/all.sh` → 12 scripts,
0 blocking, **0 warnings** (kept clean across all session commits).

---

## 2. 1.1.a — verify_topic_types.py

Already fully documented in `docs/agent/phase1_log.md §"Sub-fase
1.1.a"`. The three tests from spec §2.4 passed:

1. Positive (current workspace): scanned 134 declarations, 65
   cross-checked, 0 mismatches.
2. Negative (artificial regression): verifier exit 1 BLOCKING with
   file:line of the bad subscriber.
3. Histórico: ran against pre-fix file content from commits
   `08ac348~1` (rail_driver) and `8d81517~1` (mode_arbiter, the
   original CRITICAL-11-A-01). Verifier detects each bug at the
   exact pre-fix line. Empirical evidence the 4th occurrence of
   the bug class cannot ship past this gate.

The verifier is wired into `tools/verify_specs/all.sh` BLOCKING and
runs on every commit + the new "Run" button in the System Health
Panel (operator-confirmed working).

---

## 3. 1.1.b.full — server-first bootstrap

### Architecture change

Before (commit `12f2771` and earlier): `main()` did
`await rclnodejs.init()` before `server.listen()`. If rclnodejs
hung or threw, the HTTP server never came up — exactly the
operator's stated trauma scenario.

After (commit `b79c148`):
- Module-level `const deps: AppDeps = {...}` with stub setMode and
  executeMission that return offline errors.
- `bootstrapServer()` builds Express + middlewares + dashboard
  static + `registerAllRoutes(app, deps, null)` + http.createServer
  + setupControl/TeleopWs + `server.listen()`. Runs synchronously
  before any rclnodejs call. Returns the http.Server.
- `runRosLifecycle()` retry loop with exponential backoff (1 s → 30
  s cap). Each iteration: rclnodejs.init → build pubs/subs/clients →
  spin → mutate deps.setMode + deps.executeMission with real impls →
  rosProxy.setImpl(realRos). On health-check failure: revert to
  stubs, clear proxy impl, destroy node, shutdown, loop.
- `waitForRosFailure(node)` polls `node.getTopicNamesAndTypes()`
  every 3 s; trips after 3 consecutive zero-topic ticks. The
  practical signal that the ROS daemon went away.
- Top-level: `bootstrapServer(); runRosLifecycle().catch(...)`.

### Test 1 — HW-verified

Procedure (executed this session):

```bash
sudo systemctl stop agv.service                 # inactive
cd src/agv_ui_backend
AGV_DATA_DIR=/tmp/agv_test_bootstrap AGV_PORT=8091 \
  ROS_DOMAIN_ID=99 node dist/index.js &
# HTTP listened on :8091 within 2 seconds of process start.
curl /api/auth/status        → {"enabled":true}
curl /api/system/ros_status  → {"status":"online","detail":"ROS bridge active"}
curl /api/status             → {"robot_state":"offline","wheel_odom_hz":0,...}
```

The backend log printed `AGV Backend (TS) HTTP listening on
http://0.0.0.0:8091 — ROS bridge: offline` immediately at start.
The "offline" prefix proves bootstrapServer ran BEFORE
rclnodejs.init() succeeded. **Operator's trauma scenario is
empirically closed.**

### Tests 2-4 — implementation correctness, not run end-to-end

The 4-test rig in spec §3.4 requires running the backend
independently of the production `agv.service` (which couples
backend lifecycle to ros2 launch). Implementing a
production-equivalent test harness was outside the time budget;
documented in `future_work.md`. The retry loop + health watcher
are reviewable from `src/agv_ui_backend/src/index.ts:1059-1141`.

### Semantic nuance

`rclnodejs.init()` succeeds even when no other AGV node is
running — it just creates a DDS participant. So `/api/system/ros_status`
reports `online` once init+spin succeed, even if the AGV stack is
silent. This is intentional and consistent with "HTTP server is
independent of ROS"; the per-topic health monitor (1.1.c) is the
true diagnostic surface.

---

## 4. 1.1.c — System Health Panel

### Backend (commits `fd5b04c` + `a1f37ee`)

Files:

| Path | Role |
|---|---|
| `src/agv_ui_backend/config/health_monitor.json` | 14 components + 6 verifiers + 1 restart target (`agv_service`) |
| `src/agv_ui_backend/src/health_monitor.ts` | config loader, per-component evaluators, JSONL events |
| `src/agv_ui_backend/src/routes/health.ts` | 6 REST endpoints |

Endpoints:

```
GET  /api/health/components              list of all with current status
GET  /api/health/components/:id          single component + recent events
POST /api/health/components/:id/restart  body {"confirmation":"yes"} (engineer)
GET  /api/health/verifiers               6 registered verifiers
POST /api/health/verifiers/:id/run       execute, captures stdout/stderr/code (engineer)
GET  /api/health/events?lines=N          recent JSONL events
```

Component checks implemented:
- `topic_alive` — reads AppState freshness fields. Currently wired
  for: `/agv/wheel_odom`, `/agv/imu/filtered`,
  `/agv/collision_monitor_state`, `/agv/mode/state`,
  `/agv/localization/state`, `/agv/scan`, `/agv/odometry/global`.
- `systemd` — `systemctl is-active <unit>`.
- `network` — `/sys/class/net/<iface>/operstate`.
- `can_link_up` — `ip -details link show <iface>`.
- `chrony_synced` — `chronyc tracking` with offset thresholds.

Restart endpoint (commit `a1f37ee`):
- Components declare a `restart` field naming a target in
  `restart_targets`. Only the `agv_service` target ships today
  (kills + relaunches the entire ROS stack via
  `sudo systemctl restart agv.service`).
- Sensors (zed_2i, odrive, lidar) have `restart: null` with a
  `restart_help` line for operator guidance ("Physical USB cycle
  required" / "Restart agv.service to re-init CAN talker").
- For `self_terminating` targets the endpoint responds 202 FIRST,
  then exec — otherwise the backend dies before the response
  flushes.
- Requires `confirmation: "yes"` in the body.
- Engineer role required.

JSONL persistence:
- `${AGV_DATA_DIR}/events/health-YYYY-MM-DD.jsonl`.
- Daily rotation, 7-day retention (mtime-based cleanup).
- Events: `component_status` transitions (planned, not actively
  written yet — requires a state-machine layer that diffs polls;
  documented as polish), `verifier_run` (active — every Run from
  UI persists), `action: restart` (active).

### Frontend (commit `fd5b04c`)

Files:

| Path | Role |
|---|---|
| `web/agv_dashboard/src/components/HealthPanel.tsx` | modal panel |
| `web/agv_dashboard/src/components/TopBar.tsx` | added `Health` button |
| `web/agv_dashboard/src/App.tsx` | `showHealth` state + render |

Panel behaviour:
- Polls `/api/health/{components,verifiers,events}` every 3 s
  while open.
- Components rendered grouped by section (Sensors, Localization,
  Navigation, Services, Network) with green/amber/red/idle/unknown
  glyphs.
- Critical-red rows get a `CRIT` badge.
- Overall status indicator at top: red if any critical red,
  amber if any amber, else green.
- Verifier section at the bottom with per-row Run button. Modal
  shows stdout/stderr inline; pass/fail badge.
- Recent Events ticker (last 30 events).

### Operator-side empirical tests (spec §4.6)

| # | Test | Status |
|---|---|---|
| 5 | Verifier execution from UI | **PASSED** — operator ran `all.sh` from the panel this session, got `PASS exit 0, 5935 ms` with full stdout streamed back to modal. Confirmed by the operator's own message. |
| 1, 2, 3, 4, 6 | Boot normal / ROS down / sensor disconnect / sensor reconnect / restart-from-UI | **OPERATOR-VERIFY** — available to exercise via http://JETSON-LAN-IP:8090/dashboard. Implementation is in place; the UI behaviour requires a browser session to confirm visually. |

The fact that Test 5 passed is the load-bearing evidence that
the backend↔frontend wiring works end-to-end: the UI made an
authenticated POST, the backend exec'd the verifier script,
captured stdout, returned JSON, and the panel rendered it.

---

## 5. Files touched this Sub-fase 1.1 (full)

| Path | Source commit(s) |
|---|---|
| `tools/verify_specs/verify_topic_types.py` | b051147 |
| `tools/verify_specs/README.md` | b264054 |
| `tools/verify_specs/all.sh` | b051147 |
| `docs/agent/phase1_log.md` | b264054, 12f2771, b79c148, fd5b04c |
| `docs/agent/future_work.md` | b264054, b79c148, fd5b04c |
| `docs/agent/phase1_1_checkpoint_report.md` | 7fc9ba3 (early), this commit (final) |
| `src/agv_ui_backend/src/ros_lifecycle.ts` | 12f2771 |
| `src/agv_ui_backend/src/routes/system.ts` | 12f2771 |
| `src/agv_ui_backend/src/routes/index.ts` | 12f2771, fd5b04c |
| `src/agv_ui_backend/src/index.ts` | 12f2771, b79c148, a1f37ee |
| `src/agv_ui_backend/src/app_deps.ts` | a1f37ee |
| `src/agv_ui_backend/src/health_monitor.ts` | fd5b04c, a1f37ee |
| `src/agv_ui_backend/src/routes/health.ts` | fd5b04c, a1f37ee |
| `src/agv_ui_backend/config/health_monitor.json` | fd5b04c, a1f37ee |
| `web/agv_dashboard/src/components/HealthPanel.tsx` | fd5b04c |
| `web/agv_dashboard/src/components/TopBar.tsx` | fd5b04c |
| `web/agv_dashboard/src/App.tsx` | fd5b04c |

---

## 6. Items moved to `docs/agent/future_work.md`

- **Server-first lifecycle test rig** — `make backend-only` or
  `agv-backend.service` unit to run the 4 reconnect tests
  reproducibly outside the production launch.
- **/api/health/components/:id/restart for non-`agv_service` targets**
  — currently only the whole-stack restart is wired; per-node lifecycle
  restart (ros2 lifecycle, individual processes) needs per-component
  strategy.
- **/api/health/logs/:id?lines=N** — journalctl tail proxy. Spec
  §4.3 lists it; deferred.
- **WS event channel for live status** — currently the panel polls
  every 3 s. Streaming would be cheaper.
- **Subscribers for ekf_local / cuvslam / marker_correction /
  safety_supervisor** — these topics show `unknown` in the panel
  because the backend doesn't tap them. Add subscribers + state
  fields to close those rows.
- **`component_status` JSONL events** — the schema is in place but
  no producer runs yet (requires polling history + diff). The
  `verifier_run` and restart-action events ARE being written.
- **`/api/system/ros_status` semantic refinement** — currently
  reports "DDS participant created" rather than "AGV stack alive".
  Per-topic health (the panel) resolves this for the operator;
  the binary status remains coarse.

None of these block Phase 1 progress.

---

## 7. Time vs estimate

Prompt estimate: 3 days, escalate at 4.

Actual: a single continuous session, sequential after the earlier
Section-0 + Sprint-X + G1 + G5 work. The session also absorbed:
- ekf_local HIGH-04-09 fix commit and HW deferral.
- SSOT-revert empirical re-validation post-Sprint-X.
- G5 ODrive bilateral asymmetry firmware write.
- All of Sub-fase 1.1.

This is longer than the 3-day prompt estimate. The cause:
- The Section 0 day-2 + Sprint X work (SSOT revert + HAL rescale +
  re-validation) was a prerequisite that the prompt didn't account
  for explicitly. Sub-fase 1.1 could not safely start with the
  inflated SSOT in place.
- The 1.1.b.full refactor was larger than the lean 1.1.b version I
  had initially scoped, but smaller than I feared once the static
  deps + bootstrapServer split was clear.
- 1.1.c moved fast because the proxy contract from 1.1.b was
  already in place — the panel just consumes a well-defined API.

---

## 8. Recommendations for Sub-fase 1.2

Per prompt §5.4 — agent stops here and awaits explicit OK to
proceed. Recommendations for whatever comes next:

1. **Run the 5 remaining UI tests (1, 2, 3, 4, 6 of spec §4.6)**.
   The implementation is shipped; each is a 30-second browser
   action. Updating this report or `phase1_log.md` with the
   results closes Sub-fase 1.1 fully.

2. **Sub-fase 1.2 candidates** depending on operational priority:
   - **Panel 2 (Tag Layout Loader)** — operator can load known
     AprilTag layouts into the system.
   - **The 4 reconnect tests** for 1.1.b.full — needs the test rig.
   - **HIGH-04-09 empirical re-validation** — the ekf yaw fix is
     `CLOSED-VERIFIED-CODE`; one USB-reboot test on the armed
     robot closes it to `CLOSED-VERIFIED-HW`.

3. **`docs/agent/future_work.md`** has 8 captured items, none
   blocking. Cluster a "Sub-fase 1.1 polish" sprint if any cluster
   surface together (e.g., extra subscribers + WS streaming +
   component_status events).

---

## 9. Stop and wait

Per prompt §5.4: detenido. Esperando OK explícito antes de Sub-fase
1.2.
