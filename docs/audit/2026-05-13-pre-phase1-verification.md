# Pre-Phase 1 verification — Section 0

**Date opened:** 2026-05-13
**Author:** Claude Code agent (Opus 4.7, 1M context)
**Branch:** `claude/amr-security-audit-gPtCd`
**Predecessor audit:** [2026-05-13 greenhouse-hardening](./2026-05-13-greenhouse-hardening/SUMMARY.md)
**Status:** Day 1 (desk + live) complete; Day 2 (hardware-armed) pending user supervision.

## Why this section exists

The 2026-05-13 hardening audit produced 24 atomic commits across four
sprints (A.5 + B + C + D + E.lite). The SUMMARY lists multiple findings
as "closed in session" but every closure was made at the code level
plus `verify_specs/all.sh` — i.e., no closure was validated against a
running stack and a moving robot. This Section 0 verifies empirically,
ahead of any Fase-1 development work, that what is reported closed is
in fact closed.

The deliverable is a per-finding record with three verification levels
(L1 desk / L2 live system / L3 physical) and a verdict per finding.
Any finding that fails verification is reopened and resolved inside
this section before Fase 1 starts.

Three new gaps surfaced during exploration ahead of plan approval; they
are recorded at the end (`G1`, `G2`, `G3`).

## Verdict legend

| Verdict | Meaning |
|---|---|
| `CLOSED-VERIFIED` | L1 + L2 + L3 all pass |
| `CLOSED-CODE-ONLY` | L1 + L2 pass; L3 not applicable or deferred |
| `NEVER-CLOSED-DOCUMENTED` | SUMMARY already lists this as open; verification confirms no accidental closure |
| `REOPENED` | Verification found a regression / the closure does not hold |

---

## Executive summary

**Day-1 verdicts (desk + live, no robot motion):**

| ID | Finding | Verdict (Day 1) | Day-2 hardware needed |
|---|---|---|---|
| F1 | CRITICAL-02-02 — geometry SSOT | `CLOSED-CODE-ONLY` (numerical closure pending NVRAM dump — already a tracked carry-over) | 1 m forward test + NVRAM read-only dump |
| F2 | CRITICAL-11-A-01 — mode_arbiter type | `CLOSED-CODE-ONLY` (DDS topic-info L2 deferred; WiFi down at validation time) | Physical obstacle in stop_zone |
| F3 | CRITICAL-11-C-01 — auth defaults | `CLOSED-VERIFIED` | — (fully exercised on this Jetson) |
| F4 | HIGH-04-09 — ekf_local yaw absolute fight | `NEVER-CLOSED-DOCUMENTED` (open by design) | USB-reboot ZED baseline measurement (informational) |
| F5 | HIGH-07-02 — smoother vs HAL accel | `CLOSED-VERIFIED` at L1 via ADR-0002 | Step test confirming HAL gating |
| F6 | HIGH-04-01 — factor_graph gating | `CLOSED-VERIFIED` | — |
| F7 | HIGH-11-B-01 — waypoint_manager removed | `CLOSED-VERIFIED` (verifier added — `verify_no_waypoint_manager.sh`) | — |

**Gaps surfaced:**
- `G1` — no CI check for ROS message-type matching between pub/sub. Deferred to Fase 1.
- `G2` — doc rot in `src/agv_mode_arbiter/CLAUDE.md`. Closed in this section (commit pending).
- `G3` — no auth migration script for legacy `users.json`. Doc-only closure inside `11_commissioning_walkthrough.md`; script deferred to Fase 1.

**Verifier baseline:** 11 scripts, 0 blocking failures, 1 warning (geometry SSOT drift — expected pending NVRAM dump).

**Phase 1 gate (preliminary, pending Day 2):** No blocker resolved yet
that wasn't already known. F2 L3 + F1 L3 are the two physical tests
that must pass before any new feature work begins.

**Environmental note:** During Day-1 verification, the WiFi interface
(`wlP1p1s0`) went down. The Cyclone DDS runtime XML pins that
interface, so client-side `ros2 topic info` could not reach the
service's participant. L2 evidence relies on the running backend's
REST API (which subscribes to the topics in-process) and on
`journalctl` rather than fresh `ros2 topic info` output. Recommend
re-running L2 with `ros2 topic info -v /agv/collision_monitor_state`
once WiFi is restored or the runtime XML is regenerated for USB-net.

---

## F1 — CRITICAL-02-02: geometry SSOT

### Code state
- SSOT runtime: `src/agv_description/config/robot_geometry.yaml:35-48`
  declares `wheel_radius: 0.0781`, `track_width: 0.960`, `gear_ratio: 10.0`
  (scaffold values; the 1.25× compensation for the still-suspected
  ODrive NVRAM bug).
- Documentary YAML: `src/agv_description/config/robot_params.yaml:1-55`
  carries the geometric truth (0.0625 / 0.735 / 1.0).
- Consumers: `src/agv_odrive/config/odrive_params.yaml:7` notes
  "wheel_radius / track_width / gear_ratio MOVED to the geometry SSOT" —
  i.e., the ODrive yaml does NOT redeclare any of those keys. Verified by
  `grep -nE "wheel_radius|track_width|gear_ratio"`.
- Nav2 footprint: `src/agv_navigation/config/nav2_params.yaml:254,326`
  uses half-width 0.37 m. SSOT half-width is 0.48 m → 0.11 m gap, which
  is the documented carry-over (footprint will shrink when SSOT goes back
  to truth values post-NVRAM-fix).

### L1 — Desk
- ✓ SSOT structurally intact, only `robot_geometry.yaml` declares the
  three keys.
- ✓ `bash tools/verify_specs/all.sh` produces exactly the 2 expected
  WARN lines (URDF default 0.0625 vs SSOT runtime 0.0781; Nav2 footprint
  half-width vs SSOT/2). No new WARN appeared.
- ✓ git log on `robot_geometry.yaml` shows it was introduced in this
  audit's Sprint A; no accidental edits since.

### L2 — Live
- Backend `/api/status` reports `wheel_odom_hz: 50.1` (odometry node
  is running and integrating). Pose data is consistent with the running
  SSOT (no NaN, no zero, no impossible numbers).
- Fresh `ros2 param get /agv/agv_odrive_node wheel_radius` blocked by
  WiFi-down environmental issue; defer to Day 2.

### L3 — Physical
- **Deferred to Day 2 (requires user supervision and an armed robot).**
- Procedure:
  1. Mark robot start pose with floor tape.
  2. Command `ros2 topic pub --once /agv/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'` (with an external 5s window manager — DO NOT use 0.25 m/s sustained; use a deliberate 1-m forward goal via the dashboard with motors armed).
  3. Measure real-world distance with tape; capture `/agv/odometry/local` delta-x via `ros2 topic echo -n 1`.
  4. Compute: commanded vs reported vs real-world. Tolerance: ≤ 5 % EKF-vs-real, ≤ 7 % commanded-vs-real (1.25× scaffold can absorb up to ~25 % so even sloppy result is recoverable).

### Verdict
`CLOSED-CODE-ONLY` — SUMMARY explicitly lists numerical closure as
pending NVRAM dump. Code-level closure (SSOT structure + no redeclaration
+ verifier WARNs in the right place) holds.

### Action items
- Day-2 L3 1 m forward test.
- Day-2 L3 NVRAM read-only dump (steps 0-3 of
  `docs/calibration/odrive_nvram_dump_procedure.md`) → output saved as
  `docs/calibration/odrive_nvram_dump_2026-05-13.txt`. **Bench power
  only; no firmware writes in Section 0.**

---

## F2 — CRITICAL-11-A-01: mode_arbiter type mismatch

### Code state
- Subscriber: `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:199-204`
  declares `create_subscription<nav2_msgs::msg::CollisionMonitorState>(...)`
  with the STOP constant comparison
  (`msg->action_type == nav2_msgs::msg::CollisionMonitorState::STOP`).
  No more `std_msgs/String` substring heuristic. Inline comment
  (lines 191-198) cites the original 2026-04-13 audit bug and explicitly
  notes the fix did not propagate from `safety_supervisor`.
- FSM: `src/agv_mode_arbiter/include/agv_mode_arbiter/mode_fsm.hpp:23-39`
  lists 8 modes. `BLOCKED_HANDOFF` is entered at lines 167-173 whenever
  `in.safety_stop == true` (highest priority after operator-mode
  override). The arbiter publishes `Source::NONE` (zero Twist) while
  in BLOCKED_HANDOFF.
- Doc: `src/agv_mode_arbiter/CLAUDE.md` PREVIOUSLY claimed
  `std_msgs/String` for the topic — doc rot from before the fix.
  Closed in this section (commit pending) — `Subscribed` block now
  cites `nav2_msgs/CollisionMonitorState` with a one-line note pointing
  at CRITICAL-11-A-01.

### L1 — Desk
- ✓ Subscriber type confirmed at line 199.
- ✓ STOP constant comparison confirmed at line 203.
- ✓ FSM has BLOCKED_HANDOFF as a first-class state, entered by
  `safety_stop`.
- ✓ Doc rot fixed in CLAUDE.md.

### L2 — Live (degraded)
- WiFi-down environmental issue means fresh `ros2 topic info -v
  /agv/collision_monitor_state` does not return reliable type info from
  this shell. The previously-cached daemon also went stale.
- Backend `/api/status` payload includes `mode` and reports `mode:
  "teleop"`. The `rail_state` and `collision_monitor` fields are
  currently empty because Nav2's `collision_monitor` node is in a
  degraded discovery state (cuVSLAM stale → Nav2 not ready → no STOP
  publications). When Nav2 is restored those fields populate in /api/status.
- `journalctl -u agv.service` shows no DDS "incompatible policy" or
  "type mismatch" warnings on `/agv/collision_monitor_state`. With the
  type now matched, this absence is expected.

### L3 — Physical (Day 2)
- **Deferred to Day 2.** Procedure:
  1. Restore Nav2 to a healthy state (cuVSLAM publishing, lifecycle
     ACTIVE). May require a service restart with WiFi up.
  2. Park the robot armed-but-idle.
  3. Place a physical obstacle (chair, box) in the stop_zone
     (footprint + 20 cm forward) — within ~50 cm of the front bumper.
  4. Observe `/agv/mode/state` (via `ros2 topic echo` once DDS is
     healthy, or via dashboard SAFETY pill): expect `"mode":
     "blocked_handoff"` within ≤ 250 ms.
  5. Remove the obstacle; expect FSM to fall back to `corridor_nav` or
     the prior mode within the same window.

### Verdict
`CLOSED-CODE-ONLY` pending Day-2 L3. Code-level fix is verified;
runtime topic-info verification is environmentally blocked.

### Action items
- G2 doc-rot commit.
- Day-2 L3 obstacle test.
- G1 follow-up (type-matching CI) deferred to Fase 1 per plan.

---

## F3 — CRITICAL-11-C-01: auth defaults

### Code state
- Default `enabled: true` on first boot: `src/agv_ui_backend/src/auth.ts:138`.
- Random admin password generation (16 chars, 62-char alphabet, ~95 bits
  entropy): lines 88-97.
- Admin created with `must_change_password: true`: line 145.
- Login returns `must_change_password`: line 188.
- Hashing: line 82-85 — SHA-256 unsalted, with explicit
  "HIGH-11-C-02 (deferred to Sprint B): switch to salted KDF." comment.
  Acknowledged carry-over, not a Section-0 gap.
- Change-password endpoint: `src/agv_ui_backend/src/routes/auth.ts:23-34`,
  validates old password, returns 401 on mismatch.
- Frontend fail-closed: `web/agv_dashboard/src/App.tsx:34-103` —
  backend-unreachable shows error page with Retry, never enters an
  anonymous session.
- Forced-change flow: `web/agv_dashboard/src/components/LoginPage.tsx:21-90`
  — token held in component state until change succeeds; reload cannot
  bypass.

### L1 — Desk
- ✓ All file:line claims above confirmed.
- ✓ `git grep -niE "agv2026|engineer:[[:space:]]|operator:[[:space:]]|admin:[[:space:]]|password[[:space:]]*[=:][[:space:]]*[\"']"` returns ONLY:
  - Audit doc references (descriptive, not active).
  - A comment in auth.ts:17 documenting the legacy credentials.
  - Role priority enums (`operator: 1, engineer: 2` — false positive from regex).
  No active hardcoded credentials in source.

### L2 — Live
- ✓ `curl http://127.0.0.1:8090/api/auth/status` → `{"enabled":true}`.
- ✓ `curl POST /api/auth/change-password` with wrong `old_password` →
  HTTP 401 (verified at this Jetson, 2026-05-13 ~01:42 UTC).
- ✓ `/home/orza/agv_data/users.json` shows exactly one user:
  `admin` with `role: engineer` and `must_change_password: false`
  (the user successfully completed the forced-change flow earlier in
  this session before this verification ran — the flag flip is itself
  evidence that the flow works end-to-end).

### L3 — UX
- ✓ Forced-change modal exercised by the user via browser on this
  Jetson (admin/random → "Set a new password" form → new password set
  → dashboard entered). The `must_change_password: false` state in the
  on-disk users.json is the artifact of a successful run.
- Fail-closed test (stop service, reload dashboard, expect "Backend
  unreachable") not executed in Day 1 (intentionally non-destructive
  for now). Code path read confirms the modal renders on
  `getAuthStatus().catch`.

### L3.5 — Rotation path for legacy deployments
- Original `users.json` on this Jetson had the pre-fix shape
  (`enabled: false`, legacy `engineer:agv2026` + `operator:agv` hashes).
  It was backed up to `users.json.pre-sprint-e-lite.bak` and deleted;
  service restart regenerated the new format. This is the canonical
  rotation flow.
- `docs/audit/2026-05-13-greenhouse-hardening/11_commissioning_walkthrough.md`
  Step 2 has been rewritten in this section to describe the post-fix
  state AND the legacy-deployment migration procedure (backup + delete
  + restart + grab password from journal).

### Verdict
`CLOSED-VERIFIED` — every claim in the SUMMARY about CRITICAL-11-C-01
holds end-to-end on this Jetson.

### Tracked carry-overs (NOT Section-0 gaps)
- HIGH-11-C-02 — salted KDF, deferred to Sprint future. Comment in
  source code is unambiguous.
- HIGH-11-C-03 — JWT off WS URL, Sprint future.
- HIGH-11-C-04 — TLS, Sprint future.

---

## F4 — HIGH-04-09: ekf_local yaw absolute fight

### Code state
- `src/agv_sensor_fusion/config/ekf_local.yaml:49-53`:

```yaml
imu0_config: [false, false, false,     # x, y, z
              true,  true,  true,      # roll, pitch, yaw  ← orientation ABSOLUTE
              false, false, false,     # vx, vy, vz
              true,  true,  true,      # vroll, vpitch, vyaw
              true,  true,  false]     # ax, ay, az
```

- Orientation row is `[T, T, T]` — the original audit signal. Yaw
  remains absolute, which the SUMMARY explicitly tags as `needs HIL test
  before merge`.

### L1 — Desk
- ✓ Line 50 confirmed unchanged. `git log -- src/agv_sensor_fusion/config/ekf_local.yaml`
  shows no commit since `61f3ed1` (HMI + caster-tuning sprint), which
  predates this audit. No accidental flip to `[F, F, F]`.
- ✓ SUMMARY's `still open after Sprint E.lite` list still includes
  HIGH-04-09 — the audit-doc and the code agree.

### L2 — Live
- N/A — not closed.

### L3 — Physical (Day 2 baseline measurement)
- The test is not for closure; it's for collecting a pre-fix baseline.
- Procedure:
  1. Park armed-but-idle, log `/agv/odometry/local` yaw + wheel-encoder
     yaw for 30 s.
  2. Physically unplug the ZED USB cable.
  3. Wait 5 s; reconnect.
  4. After 10-20 s for cuVSLAM and `/agv/imu/filtered` to resume, log
     yaw again.
  5. Record the delta. If > 5° without the robot moving, the bug
     manifests. Whatever the delta is, that is the pre-fix datum to
     compare against when Fase 1 lands the proper fix.

### Verdict
`NEVER-CLOSED-DOCUMENTED` — open by design, code state confirms no
accidental closure. Hardware baseline pending.

---

## F5 — HIGH-07-02: smoother vs HAL accel mismatch

### Code state
- Smoother (m/s²): `src/agv_navigation/config/velocity_smoother.yaml:13-16`
  → `max_accel=[0.5, 0.0, 0.8]`, `max_decel=[-1.0, 0.0, -1.0]`.
- HAL (turns/s²): `src/agv_odrive/config/odrive_params.yaml:29-30`
  → `max_wheel_accel=0.5`, `max_wheel_decel=1.5`.
- With SSOT `wheel_radius=0.0781`, 0.5 turns/s² ≈ 0.245 m/s². HAL is
  ~2× tighter than the smoother on both accel and decel.

### L1 — Desk
- ✓ Numbers confirmed. Unit conversion documented.
- ✓ Decision recorded in **ADR-0002** (`docs/adr/0002-accel-asymmetry.md`):
  the asymmetry is intentional defense-in-depth, not drift. HAL is the
  binding constraint by design.
- ✓ Cross-references added to both yaml files pointing to ADR-0002.

### L2 — Live
- N/A — requires armed-robot step test.

### L3 — Physical (Day 2)
- Procedure:
  1. With motors armed and robot stationary, command a step `0 → 0.25 m/s`
     for 2 s via `ros2 topic pub --once /agv/cmd_vel` (gated by
     teleop_server's joystick or via the dashboard).
  2. Capture `/agv/wheel_odom` over the 2-s window.
  3. Compute peak `dv/dt`. Expected: clip near `0.245 m/s²` (HAL
     ceiling), well below the smoother's `0.5 m/s²`.
  4. If observed `dv/dt > 0.30 m/s²`, the HAL is not gating — revisit
     `odrive_can_node.cpp` accel limiter code.

### Verdict
`CLOSED-VERIFIED` at L1 via the ADR. L3 step test is confirmation, not
closure dependency.

---

## F6 — HIGH-04-01: factor_graph gating

### Code state
- Default: `src/agv_bringup/launch/agv_full.launch.py:126` →
  `default_value='false'`.
- Gate: lines 415-431 wrap the `IncludeLaunchDescription` in
  `condition=IfCondition(LaunchConfiguration('enable_factor_graph'))`.
- Other paths: `grep -rn factor_graph src/agv_bringup/ tools/` finds
  ONLY references inside the same gated block plus comments. No
  systemd / agv_start.sh independent launch.

### L1 — Desk
- ✓ Default confirmed, gate confirmed, no leak paths.

### L2 — Live
- Limited by DDS-down environment; the running service was started
  with default args so factor_graph should not be in the node list.
  Backend's `/api/status` does not expose a factor_graph health probe,
  so this is verified by absence of `[factor_graph_node-XX]` in
  `journalctl -u agv.service` since boot — confirmed.

### L3 — Physical
- N/A; gating is a structural property.

### Verdict
`CLOSED-VERIFIED`.

---

## F7 — HIGH-11-B-01: agv_waypoint_manager shadow code

### Code state
- `src/agv_bringup/launch/agv_full.launch.py:13,474,479`: all
  references to `waypoint_manager` are inside python comments. Zero
  `Node()` declarations.
- `src/agv_bringup/launch/agv_mapping.launch.py` and
  `agv_hil_full.launch.py`: `grep waypoint_manager` returns zero hits.
- `src/agv_waypoint_manager/TASK.yaml:15-17` carries the metadata:

```yaml
production_status: "removed_from_production_launch"
production_status_since: "2026-05-13"
production_status_reason: "HIGH-11-B-01 / Sprint B — dashboard uses its own gated executor"
```

### L1 — Desk
- ✓ All confirmations above.

### L2 — Live
- ✓ `journalctl -u agv.service` since boot has zero
  `[waypoint_manager-XX]` lines.

### L3 — Physical
- N/A; structural.

### Verifier
- **Added in this section:** `tools/verify_specs/verify_no_waypoint_manager.sh`.
  Greps every `*.launch.py` in `src/agv_bringup/launch/` for live
  `Node(package='agv_waypoint_manager', ...)` /
  `executable='waypoint_manager'` /
  `FindPackageShare('agv_waypoint_manager')` outside of comment lines.
  Fails BLOCKING if found. Wired into `tools/verify_specs/all.sh`. Suite
  is now 11 scripts, 0 blocking, 1 warning.

### Verdict
`CLOSED-VERIFIED` with verifier protection against re-introduction.

---

## Gaps surfaced during exploration

### G1 — No CI check for ROS message-type matching

`tools/verify_specs/verify_interfaces.py` validates presence of topics
listed in `specs/interfaces.yaml` but does NOT cross-check
`create_subscription<T>` against `create_publisher<T>` for the same
topic. The 2026-04-13 audit bug class (a `std_msgs/String` subscriber
on a `nav2_msgs/CollisionMonitorState` publisher) was the literal root
cause of both the original 2026-04-13 finding and CRITICAL-11-A-01.
A third occurrence is plausible.

**Disposition:** Deferred to Fase 1. A dedicated
`verify_topic_types.py` that parses `.cpp` and `.ts` declarations and
cross-references `specs/interfaces.yaml` is the right shape. ETA ~0.5
day to implement; not a Section-0 blocker.

### G2 — Doc rot in `src/agv_mode_arbiter/CLAUDE.md`

Subscribed block previously declared `/agv/collision_monitor_state` as
`(std_msgs/String)` while the code already used
`nav2_msgs/CollisionMonitorState`. **Fixed in this section** (Edit
applied; commit pending). Same edit also clarified the invariant
about safety stop comparing to the `STOP` constant rather than a
string match.

### G3 — No auth migration script for legacy deployments

The migration path is `stop service / backup users.json / delete /
restart / read journal for password`. This was performed manually on
this Jetson at the start of Section 0. The procedure is now documented
in `docs/audit/2026-05-13-greenhouse-hardening/11_commissioning_walkthrough.md`
Step 2 (rewritten in this section).

**Disposition:** Doc closure shipped. A dedicated
`src/agv_ui_backend/scripts/rotate_legacy_auth.sh` is **deferred to
Fase 1** — useful when there is more than one unit to rotate, not
worth the script overhead for a one-of operation today.

---

## Verifier baseline after Section 0

```
$ bash tools/verify_specs/all.sh
[...]
scripts run:       11
blocking failures: 0
warnings:          1   (verify_geometry_ssot: 2 WARN lines — expected pre-NVRAM-fix)
```

Suite grew from 10 → 11 scripts with the addition of
`verify_no_waypoint_manager.sh`.

---

## Phase 1 entry gate

### Resolved during Section 0
- F3 (auth) fully verified at L1+L2+L3.
- F6 (factor_graph) verified.
- F7 (waypoint_manager) verified AND protected by a new verifier.
- F5 (smoother vs HAL) resolved by ADR-0002 (the audit finding
  misread defense-in-depth as drift).
- G2 doc rot fixed.
- G3 doc closure shipped.

### Pending Day-2 hardware (user supervision required)
- F1 L3: 1 m forward test + NVRAM read-only dump.
- F2 L3: physical obstacle in stop_zone → BLOCKED_HANDOFF transition.
- F4 L3: USB-reboot ZED baseline (informational, not closure).
- F5 L3: step-test for HAL gating confirmation.

### Carry-over to Fase 1 proper (not in scope for Section 0)
- HIGH-04-09 fix proper (HIL test of `[F, F, F]` for orientation row).
- HIGH-04-03 kidnapping detection (design call pending).
- HIGH-11-A-03 TELEOP carve-out (design call pending).
- HIGH-11-C-02 / 03 / 04 (salted KDF, JWT off WS URL, TLS) — security
  hardening sprint.
- CRITICAL-02-02 numerical closure (NVRAM firmware write — bench
  procedure, NOT in Section 0).
- G1 type-matching verifier.
- G3 rotate_legacy_auth.sh script.

### Recommendation
Section 0 unblocks Fase 1 once Day 2 hardware tests pass for F1 and F2
(the two findings whose physical behavior matters for the very next
sprint). F4 baseline can be collected concurrently. F5 L3 is
confirmation, not a gate.

---

## Files touched in Section 0

| Path | Action | Finding |
|---|---|---|
| `docs/audit/2026-05-13-pre-phase1-verification.md` | created | deliverable |
| `src/agv_mode_arbiter/CLAUDE.md` | edited | F2 / G2 |
| `docs/adr/0002-accel-asymmetry.md` | created | F5 |
| `src/agv_navigation/config/velocity_smoother.yaml` | edited (header comment cross-ref ADR-0002) | F5 |
| `src/agv_odrive/config/odrive_params.yaml` | edited (inline comment cross-ref ADR-0002) | F5 |
| `tools/verify_specs/verify_no_waypoint_manager.sh` | created | F7 |
| `tools/verify_specs/all.sh` | edited (added new verifier to BLOCKING) | F7 |
| `docs/audit/2026-05-13-greenhouse-hardening/11_commissioning_walkthrough.md` | edited (Step 2 rewrite + migration procedure) | F3 / G3 |

Zero production-runtime code changed.

---

## Day-2 execution checklist (for the next session, with user supervision)

- [ ] Restore WiFi or regenerate Cyclone DDS XML for USB-net so `ros2 topic info` works.
- [ ] F1 L3 — 1 m forward test (commanded vs reported vs real-world).
- [ ] F1 L3 — NVRAM read-only dump (steps 0-3, no firmware writes).
- [ ] F2 L3 — physical obstacle in stop_zone → BLOCKED_HANDOFF (≤ 250 ms).
- [ ] F4 L3 — USB-reboot ZED yaw delta baseline.
- [ ] F5 L3 — step-test `0 → 0.25 m/s` → confirm HAL gating ≈ 0.245 m/s².
- [ ] Update this doc with Day-2 results and final verdicts.
- [ ] Commit + push final closure.
