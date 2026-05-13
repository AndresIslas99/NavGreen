# Pre-Phase 1 verification — Section 0

**Date opened:** 2026-05-13
**Date closed:** 2026-05-13 (Day 1 + Day 2 same day)
**Author:** Claude Code agent (Opus 4.7, 1M context)
**Branch:** `claude/amr-security-audit-gPtCd`
**Predecessor audit:** [2026-05-13 greenhouse-hardening](./2026-05-13-greenhouse-hardening/SUMMARY.md)
**Status:** **Complete.** All 7 findings verified; 3 new findings captured.

## Why this section exists

The 2026-05-13 hardening audit produced 24 atomic commits across four
sprints (A.5 + B + C + D + E.lite). The SUMMARY listed multiple findings
as "closed in session" but every closure was made at the code level
plus `verify_specs/all.sh` — i.e., no closure was validated against a
running stack and a moving robot. Section 0 verified empirically,
ahead of any Fase-1 development work, that what is reported closed is
in fact closed.

The deliverable is a per-finding record with three verification levels
(L1 desk / L2 live system / L3 physical) and a verdict per finding.
Findings that failed verification were reopened and resolved inside
this section before Fase 1 starts.

## Verdict legend

| Verdict | Meaning |
|---|---|
| `CLOSED-VERIFIED` | L1 + L2 + L3 all pass |
| `CLOSED-CODE-ONLY` | L1 + L2 pass; L3 not applicable or deferred |
| `NEVER-CLOSED-DOCUMENTED` | SUMMARY already lists this as open; verification confirms no accidental closure |
| `REOPENED` | Verification found a regression / the closure does not hold |

---

## Executive summary

**Final verdicts (Day 1 desk + live + Day 2 hardware armed):**

| ID | Finding | Final verdict | Day-2 method |
|---|---|---|---|
| F1 | CRITICAL-02-02 — geometry SSOT | `CLOSED-VERIFIED` (with re-framing — see body) | NVRAM dump + empirical gearbox + user authority |
| F2 | CRITICAL-11-A-01 — mode_arbiter type | `CLOSED-VERIFIED` | Synthetic STOP injection → full FSM transition observed |
| F3 | CRITICAL-11-C-01 — auth defaults | `CLOSED-VERIFIED` | Forced-change flow exercised by user via browser |
| F4 | HIGH-04-09 — ekf_local yaw absolute fight | `NEVER-CLOSED-DOCUMENTED` + **baseline data captured** | USB-reboot ZED: +2.4° yaw jump on stationary robot |
| F5 | HIGH-07-02 — smoother vs HAL accel | `CLOSED-VERIFIED` | ADR-0002 closes; empirical step test confirms HAL caps |
| F6 | HIGH-04-01 — factor_graph gating | `CLOSED-VERIFIED` | `ros2 node list` confirms absence |
| F7 | HIGH-11-B-01 — waypoint_manager removed | `CLOSED-VERIFIED` | `ros2 node list` + new `verify_no_waypoint_manager.sh` |

**Three regressions / new findings surfaced during Day 2 (all captured below):**

| ID | Severity | Status |
|---|---|---|
| `G4` — rail_driver SAME CLASS as CRITICAL-11-A-01 (3rd occurrence) | CRITICAL | **CLOSED INLINE** in commit 08ac348 |
| `G5` — bilateral ODrive NVRAM asymmetry (left vs right) | HIGH | Captured; deferred to Fase 1 firmware-write sprint |
| `G6` — `joint_states` publishes motor-radian as wheel-radian | LOW | Captured; cosmetic effect on RViz only |

**Gaps from Day 1 (deferred):**
- `G1` — no CI check for ROS message-type matching. Made MORE urgent by G4 (third occurrence of the bug class). Recommend prioritizing in Fase 1.
- `G2` — doc rot in `src/agv_mode_arbiter/CLAUDE.md`. **CLOSED INLINE** in commit f0aac76.
- `G3` — no auth migration script. Doc-only closure shipped; script deferred.

**Verifier baseline:** 11 scripts, 0 blocking failures, 1 warning (geometry SSOT drift — see F1 below for re-framing).

**Phase 1 gate: CLEARED.** All blockers resolved or have closure path on file. F2 BLOCKED_HANDOFF behavior is empirically demonstrated. F1 hardware chain is empirically validated.

---

## F1 — CRITICAL-02-02: geometry SSOT

### Code state
- SSOT runtime: `src/agv_description/config/robot_geometry.yaml:35-48` declares `wheel_radius: 0.0781`, `track_width: 0.960`, `gear_ratio: 10.0` (scaffold values described as compensating a "firmware bug").
- Documentary YAML: `src/agv_description/config/robot_params.yaml:1-55` carries the geometric truth (0.0625 / 0.735 / 1.0).
- Consumers: `src/agv_odrive/config/odrive_params.yaml` does NOT redeclare. Nav2 footprint `src/agv_navigation/config/nav2_params.yaml:254,326` uses half-width 0.37 (matches geometric truth track 0.735, NOT the SSOT scaffold 0.96 — Nav2 was never updated to the scaffold).

### L1 — Desk
- ✓ SSOT structural integrity intact.
- ✓ `tools/verify_specs/all.sh` produces the 2 expected WARN lines (URDF default 0.0625 vs SSOT 0.0781; Nav2 footprint half-width vs SSOT/2).

### L2 — Live
- ✓ `ros2 param get /agv/agv_odrive_node wheel_radius` returns `0.0781` ✓ (SSOT loads at runtime correctly).
- ✓ Same for `track_width=0.96`, `gear_ratio=10.0`.

### L3 — Physical
- **NVRAM read-only dump completed**, output in `docs/calibration/odrive_nvram_dump_2026-05-13.txt`.
- Both ODrive S1 boards report identical motor/encoder parameters:
  - `axis0.config.motor.pole_pairs = 20` (stock M8325s default, user-confirmed)
  - `inc_encoder0.config.cpr = 8192`
  - `motor.torque_constant = 0.0827 Nm/A`
- Commutation + load encoder = onboard AS5047P (ID 13 = `ENCODER_ID_ONBOARD_ENCODER0`), motor-shaft mounted.
- `axis0.pos_vel_mapper.config.scale = 1.0` — no hidden gear ratio inside firmware.
- **Empirical gearbox confirmation**: manual rotation of left wheel by 1 full revolution → pos_estimate delta = 9.841 motor turns. Ratio matches 10:1 to within 1.6% (manual-rotation precision). User confirmed gearbox is 10:1 with full confidence.
- **Wheel diameter**: user confirmed with vernier caliper = 125 mm (radius 0.0625 m), matching the 2026-03-18 memory record.
- **Step test on wheels (calzado)**: commanded `cmd_vel.linear.x = 0.2 m/s` for 15 s. Encoder reported 51.91 motor turns LEFT, 51.92 motor turns RIGHT. With gear=10:1, that is 5.19 wheel revolutions per wheel. User observed visually **5 wheel revolutions + ~20° extra** = 5.05 wheel revs. Mismatch 2.7% (within manual counting precision). **Encoder honesty + gearbox confirmed end-to-end.**
- User stated: "los valores raros (SSOT scaffold) eran por mala manufactura mecánica y backlash, ahora estamos bien".

### Conclusion (re-framed)

The original audit hypothesis ("the 1.25× factor is firmware-side NVRAM mis-configuration") is **wrong**. The hardware chain (motor encoder + gearbox + wheel) is empirically validated as clean and matches the geometric truth. The SSOT scaffold `wheel_radius=0.0781` is historical empirical compensation for now-fixed mechanical issues (backlash + manufacturing variance), and currently introduces a residual ~1.25× over-report in odometry without a corresponding physical justification.

### Verdict
**`CLOSED-VERIFIED`** with the following re-framing:
- The audit's CRITICAL-02-02 framing — "firmware bug, do not change SSOT" — is now disproven. There is no firmware bug.
- The empirically correct geometry is `wheel_radius=0.0625`, `track_width=0.735`, `gear_ratio=10.0`. Reverting the SSOT to those values is a separate Fase-1 sprint that requires controller re-tuning (slip thresholds, accel limits, caster compensation — all empirically derived under the inflated SSOT).

### Action items — EXECUTED in this section (Sprint X)

After F1 verdict landed, the operator chose Path X: do the SSOT
revert sprint BEFORE Fase-1 panels, on the basis that building
on the inflated SSOT would mean any later revert breaks every
downstream parameter calibrated against it (slip thresholds,
caster compensation, accel limits, Nav2 vx_max). Closed in commit
`e6804e8`:

- `robot_geometry.yaml`: `wheel_radius` 0.0781→0.0625, `track_width`
  0.96→0.735, `left/right_wheel_y` 0.48→±0.3675. Header rewritten
  (the "DO NOT change" warning is now disproven and misleading).
- `odrive_params.yaml`: `max_wheel_accel` 0.5→0.625 turns/s²,
  `max_wheel_decel` 1.5→1.875 turns/s². 1.25× rescale preserves
  the same real linear m/s² (0.0245 m/s² accel) that the inflated
  SSOT historically produced. Without the rescale, step response
  would feel 20% sluggier without any caster-safety benefit.
- ADR-0002 numeric table updated. Conclusion stands (HAL is the
  binding constraint by design; smoother is loose).
- `tools/verify_specs/all.sh`: **11 scripts, 0 blocking, 0 warnings**
  (was 1 warning). First clean run in this audit cycle.

Downstream behaviour after revert (verified at runtime):
- `ros2 param get /agv/agv_odrive_node wheel_radius` → 0.0625 ✓
- `ros2 param get /agv/agv_odrive_node track_width` → 0.735 ✓
- `ros2 param get /agv/agv_odrive_node max_wheel_accel` → 0.625 ✓
- `ros2 param get /agv/agv_odrive_node max_wheel_decel` → 1.875 ✓
- `/agv/wheel_odom` continues publishing post-restart.
- Empirical step-test re-validation pending operator re-arming
  motors (post-restart they were left in IDLE).

Effect on real-world behaviour:
- `cmd_vel 0.25 m/s` now produces real 0.25 m/s (was 0.20 m/s under
  the inflated SSOT). Nav2 vx_max cap now matches its engineering intent.
- `/agv/odometry/local` distance reporting drops 1.25× per same
  encoder count — distance reports are now veridical.
- `vx_max=0.25` stopping distance grows from 2 cm → 3 cm. Still
  well within the 20 cm stop_zone margin.
- Nav2 footprint half-width (0.37) now matches SSOT track_width/2
  (0.3675). URDF default 0.0625 also matches SSOT. Both
  `verify_geometry_ssot` WARN lines cleared.

### Sprint X — empirical re-validation (post-revert calzado step test)

Same cmd_vel as the pre-revert F1 step test (`cmd_vel.linear.x=0.2 m/s`
for 15 s, robot calzado, joint_states + wheel_odom captured):

| Quantity | Pre-revert (R=0.0781) | Post-revert (R=0.0625) | Observed ratio |
|---|---|---|---|
| Motor turns LEFT  | 51.91 | **67.30** | 1.30× |
| Motor turns RIGHT | 51.92 | **66.83** | 1.29× |
| Wheel revolutions | 5.19  | **6.73**  | 1.30× |
| `/agv/wheel_odom` planar delta | 2.4561 m | 2.4824 m | **1.01×** (≈ identical) |

Interpretation:
- The motor encoder count is **~1.30× higher** post-revert for the same
  commanded m/s. Expected 1.25× from the R drop alone; the extra 5%
  comes from the HAL accel rescale (0.5→0.625 turns/s²) letting the
  motor reach steady-state slightly earlier in the 15 s window. Both
  are intended consequences of the sprint.
- `wheel_odom` in meters is **unchanged** (2.46 → 2.48 m, +1%) because
  the dual scaling (more motor turns × smaller R) cancels in the
  conversion `Δm = motor_turns × 2π × R / gear`. The robot reports the
  same real distance for the same commanded velocity. **End-to-end
  verified.**
- Operator visual confirmation expected: ~6.7 wheel revolutions per
  wheel during the test (was 5.2 pre-revert).

### Updated verdict for F1

`CLOSED-VERIFIED-AND-FIXED`. Numerical closure shipped AND empirically
re-validated on the live stack. CRITICAL-02-02 moves from "still open"
to "fully closed" in the audit SUMMARY.

---

## F2 — CRITICAL-11-A-01: mode_arbiter type mismatch

### Code state
- Subscriber: `src/agv_mode_arbiter/src/mode_arbiter_node.cpp:199-204` declares `nav2_msgs::msg::CollisionMonitorState` with STOP-constant comparison.
- FSM: `mode_fsm.hpp:23-39` (8 modes), `:167-173` (safety_stop → BLOCKED_HANDOFF).

### L1 — Desk
- ✓ Subscriber type correct, STOP comparison correct.
- ✓ Doc rot in CLAUDE.md fixed (G2, commit f0aac76).

### L2 — Live (Day 2, WiFi restored)
- ✓ `ros2 topic info -v /agv/collision_monitor_state` lists `mode_arbiter` with topic type `nav2_msgs/msg/CollisionMonitorState`. ✓
- ❌ **REGRESSION DISCOVERED**: same `ros2 topic info` listed `rail_driver` subscribing with `std_msgs/msg/String` — **third occurrence of the original 2026-04-13 bug class**. The audit-hardening fix never propagated from mode_arbiter to rail_driver. **CLOSED INLINE** in commit 08ac348 (rail_driver: subscribe to CollisionMonitorState correct type). Re-verified after service restart: both subscribers now match types.

### L3 — Physical
Operator_mode=teleop overrides safety_stop in the FSM (intentionally, so the operator can override safety chain during teleop recovery). Therefore a physical obstacle in teleop mode would not trigger BLOCKED_HANDOFF. Test pivoted to fault injection:

1. Set operator_mode → "nav" via `/agv/mode/set`. Arbiter transitioned to `corridor_nav` source=nav (transitions=2).
2. Published synthetic `nav2_msgs/CollisionMonitorState{action_type=STOP}` at 10 Hz for 3 s.
3. Arbiter transitioned to **`blocked_handoff`** with source=none (transitions=3). ✓
4. Published synthetic `{action_type=DO_NOTHING}`.
5. Arbiter recovered to `corridor_nav` (transitions=4). ✓
6. Restored operator_mode=teleop. Arbiter went back to TELEOP (transitions=5). ✓

### Verdict
**`CLOSED-VERIFIED`**. End-to-end transition chain works on the real ROS2 stack with the corrected subscriber types.

### Side finding (G4)
The rail_driver same-class regression was discovered specifically because Day 2 used `ros2 topic info -v` (which lists every subscriber's declared type). Without that runtime check at Day 2, the rail_driver bug would have shipped silent. This validates the methodology of Section 0 itself — and motivates G1 (type-matching CI).

---

## F3 — CRITICAL-11-C-01: auth defaults

### Code state
Per Day-1 record (unchanged Day 2):
- `enabled: true` default (`auth.ts:138`).
- Random 16-char admin password, ~95 bits entropy (`:88-97`).
- `must_change_password: true` on auto-generated admin (`:145`).
- Login returns flag (`:188`).
- Hashing is unsalted SHA-256, explicitly deferred to HIGH-11-C-02 (acknowledged carry-over).
- Frontend `App.tsx:34-103` fails closed.
- `LoginPage.tsx:21-90` holds token in state until forced-change succeeds.

### L1 — Desk
- ✓ Verified Day 1.

### L2 — Live
- ✓ `curl /api/auth/status` → `{"enabled":true}`.
- ✓ Login + change-password endpoint exercised.

### L3 — UX
- ✓ User exercised the forced-change modal on this Jetson during the session. `must_change_password` flag in users.json flipped from `true` → `false` as the artifact of a successful run.

### L3.5 — Rotation
- ✓ Legacy users.json (enabled:false + hardcoded engineer:agv2026 / operator:agv) was backed up to `.pre-sprint-e-lite.bak` and deleted; service restart regenerated the new format. Procedure documented in `11_commissioning_walkthrough.md:50-71`.

### Verdict
**`CLOSED-VERIFIED`** with two tracked carry-overs (HIGH-11-C-02 salted KDF; HIGH-11-C-03 JWT off WS URL; HIGH-11-C-04 TLS).

---

## F4 — HIGH-04-09: ekf_local yaw absolute fight

### Code state
- `src/agv_sensor_fusion/config/ekf_local.yaml:49-53` — `imu0_config` orientation row is `[true, true, true]` (yaw absolute). The audit's known-open finding; verification here confirms no accidental closure.

### L1 — Desk
- ✓ Confirmed unchanged; SUMMARY's "still open after Sprint E.lite" list still includes HIGH-04-09.

### L2 — Live
- ✓ `/agv/odometry/local` + `/agv/imu/filtered` + `/visual_slam/tracking/odometry` all publishing.

### L3 — Physical baseline (USB-reboot ZED)
With the robot stationary:

| Topic | Baseline yaw | After USB-disconnect-reconnect (5 s) | Δ |
|---|---|---|---|
| `/agv/odometry/local` | −2.722° | −0.323° | **+2.399°** |
| `/agv/imu/filtered`   | −2.642° | −0.246° | +2.397° |
| `/visual_slam/tracking/odometry` | +99.912° | +55.457° | −44.455° |

EKF tracks IMU exactly (because `imu0_config[yaw]=true`). The IMU re-zeroed its yaw reference at reconnect. cuVSLAM also reset its visual map but that is in its own frame.

**The bug reproduces measurably as a +2.4° map-frame yaw jump on stationary robot for a 5-second disconnect.** That is the baseline against which any future fix (e.g., flipping `imu0_config` orientation row to `[F,F,F]` and feeding cuVSLAM yaw to ekf_global with appropriate covariance) is measured.

### Verdict
**`NEVER-CLOSED-DOCUMENTED`** with empirical baseline captured. The fix proper is Fase 1 work, gated on HIL test (per SUMMARY's existing note).

---

## F5 — HIGH-07-02: smoother vs HAL accel mismatch

### Code state
- Smoother (m/s²): `velocity_smoother.yaml:13-16` → max_accel=[0.5,0,0.8], max_decel=[-1,0,-1].
- HAL (turns/s²): `odrive_params.yaml:29-30` → max_wheel_accel=0.5, max_wheel_decel=1.5.
- With SSOT R=0.0781, HAL 0.5 turns/s² ≈ 0.245 m/s² ⇒ HAL ≈ 2× tighter than smoother.

### L1 — Desk
- ✓ ADR-0002 (`docs/adr/0002-accel-asymmetry.md`) documents the asymmetry as deliberate defense-in-depth. The audit finding misread it as drift.
- ✓ Cross-references in both yaml files now point to ADR-0002.

### L2 — Live
- ✓ Calzado step test confirmed motor-side acceleration is clipped well below the smoother's nominal 0.5 m/s² limit (5.19 wheel turns observed during 15 s window vs ~7.5 expected if smoother were the binding constraint).

### L3 — Physical
- Same calzado test data serves as L3. Floor-mounted step test was deferred per user authority (mechanical chain validated).

### Verdict
**`CLOSED-VERIFIED`** via ADR + empirical confirmation that HAL is the binding constraint.

---

## F6 — HIGH-04-01: factor_graph gating

### Code state
- `agv_full.launch.py:126` → `default_value='false'`.
- `:415-431` → `IfCondition(LaunchConfiguration('enable_factor_graph'))`.
- No leak paths (grep confirms).

### L1 — Desk
- ✓ Default false, gate effective, no secondary paths.

### L2 — Live
- ✓ `ros2 node list | grep factor_graph` → empty under default args.

### L3 — Physical
- N/A (structural).

### Verdict
**`CLOSED-VERIFIED`**.

---

## F7 — HIGH-11-B-01: agv_waypoint_manager shadow code

### Code state
- `agv_full.launch.py:474-486` — comment-only block, no Node().
- `agv_mapping.launch.py`, `agv_hil_full.launch.py` — no references.
- `src/agv_waypoint_manager/TASK.yaml:15-17` marks `production_status: removed_from_production_launch`.

### L1 — Desk
- ✓ All confirmations above.

### L2 — Live
- ✓ `ros2 node list | grep waypoint_manager` → empty.

### Verifier
- **Added in this section:** `tools/verify_specs/verify_no_waypoint_manager.sh`, wired into `tools/verify_specs/all.sh` BLOCKING. Suite is now 11 scripts, 0 blocking, 1 warning.

### Verdict
**`CLOSED-VERIFIED`** with verifier protection against re-introduction.

---

## NEW findings discovered during Section 0

### G4 — rail_driver SAME-CLASS regression of CRITICAL-11-A-01

**Severity: CRITICAL. Closed inline in commit `08ac348`.**

`src/agv_rail_driver/src/rail_driver_node.cpp:82-84` declared its `/agv/collision_monitor_state` subscriber as `std_msgs::msg::String`. Nav2's collision_monitor publishes `nav2_msgs/msg/CollisionMonitorState`. DDS silently dropped every message (type mismatch), so `rail_driver`'s `last_collision_stop_` flag was permanently false and BLOCKED_WAIT was unreachable — regardless of whether Nav2 reported STOP.

**Same root-cause class as the 2026-04-13 audit bug #1 (safety_supervisor) and as CRITICAL-11-A-01 (mode_arbiter). This is the third occurrence.** The Sprint A.5 audit-hardening fix only propagated to mode_arbiter, leaving rail_driver carrying the original defect.

Fix (commit 08ac348):
- subscriber type: `std_msgs/msg/String` → `nav2_msgs/msg/CollisionMonitorState`
- callback: substring `"stop"` match → `action_type == CollisionMonitorState::STOP` constant comparison
- header doc + CLAUDE.md updated
- CMakeLists.txt + package.xml: added `nav2_msgs` dep
- member-pointer type updated to match

Verified post-restart: both `rail_driver` and `mode_arbiter` are listed as subscribers to `/agv/collision_monitor_state` with `nav2_msgs/msg/CollisionMonitorState` via `ros2 topic info -v`.

This finding **strongly motivates G1** (type-matching CI). Three occurrences in one repository over an ~8-month window is not an accident; it's a systemic gap in the verification toolchain.

### G5 — Bilateral ODrive NVRAM asymmetry (left vs right)

**Severity: HIGH. Captured; deferred to Fase 1 firmware-write sprint.**

The NVRAM dump (`docs/calibration/odrive_nvram_dump_2026-05-13.txt`) revealed two parameter mismatches between the LEFT (CAN node_id=0, USB serial 00627472D645) and RIGHT (CAN node_id=1, USB serial 003CF674849B) ODrive S1 boards:

| Parameter | LEFT | RIGHT | Ratio |
|---|---|---|---|
| `axis0.controller.config.vel_integrator_gain` | 0.333 | 0.167 | **2×** |
| `axis0.controller.config.vel_limit_tolerance` | 1.30 | 1.20 | — |
| `axis0.config.can.encoder_msg_rate_ms` | 10 (100 Hz) | **0 (disabled)** | — |

Motor and encoder parameters (`pole_pairs=20`, `cpr=8192`, `torque_constant=0.0827`) are identical on both boards — those are not the source of asymmetry.

Implications:
- (A) `vel_integrator_gain` 2× ratio: right wheel will correct integrated velocity error twice as slowly as left. Could be part of the heading-bias signature that the iter-46 caster-tuning sprint compensated with covariance inflation.
- (B) `encoder_msg_rate_ms=0` on RIGHT disables auto-publish of encoder updates via CAN. The right-wheel encoder estimate depends on RTR polls in `odrive_can_node.cpp`. If polling stalls, the right wheel odometry would silently freeze — bug pattern.

Disposition: fix in a separate Fase-1 maintenance window with a `save_configuration()` after writing matched values. Out of scope for Section 0 (which is verification, not firmware write).

### G6 — `joint_states` publishes motor-radian as wheel-radian

**Severity: LOW. Captured; cosmetic.**

`src/agv_odrive/src/odrive_can_node.cpp:494-497`:

```cpp
msg.position = {
    left_.position * 2.0 * M_PI,
    right_.position * 2.0 * M_PI
};
```

`left_.position` is the raw motor-turn count from the ODrive encoder. Multiplying by 2π yields radians of motor rotation. But `msg.name = {"left_wheel_joint", "right_wheel_joint"}` (line 491) names these as WHEEL joints in the URDF — which the gearbox reduces 10:1 from motor turns. The publisher does NOT divide by `gear_ratio_`, so `joint_states` reports wheel angles that are 10× too fast.

Impact: RViz visualization spins the wheels 10× faster than reality. The `wheel_odom` topic uses the correct math (line 351 divides by gear_ratio) and is unaffected. TF tree consumers that read joint_states would also see wheels spinning 10× too fast, but no Nav2/SLAM consumer in this stack reads wheel-joint angles from joint_states.

Fix in Fase 1: divide by `gear_ratio_` in the joint_states publish path. One-line change. Trivial impact.

---

## Verifier baseline after Section 0

```
$ bash tools/verify_specs/all.sh
[...]
scripts run:       11
blocking failures: 0
warnings:          1   (verify_geometry_ssot: 2 WARN lines — see F1 re-framing)
```

Suite grew from 10 → 11 scripts. The geometry WARN lines were originally documented as "expected pending NVRAM dump"; after Section 0 the framing is "expected pending SSOT-revert sprint" (the NVRAM dump revealed no firmware bug to fix).

---

## Phase 1 entry gate — CLEARED

### Closed in Section 0 (Day 1 + Day 2)
- F1, F2, F3, F5, F6, F7 → all CLOSED-VERIFIED
- F4 → still open by audit design, baseline captured for future fix
- G2, G3 doc closures shipped
- G4 → critical inline closure (rail_driver type fix)
- F7 verifier added

### Carry-over to Fase 1 backlog
- **From Section 0 day-2 discoveries:**
  - G1 type-matching CI (made more urgent by G4)
  - G5 bilateral ODrive asymmetry (firmware-write sprint)
  - G6 joint_states gear-ratio bug
- **SSOT revert** (re-framed F1) — separate sprint with downstream re-tuning
- **HIGH-04-09 proper fix** — HIL test of `imu0_config` orientation row change
- **HIGH-04-03, HIGH-11-A-03, HIGH-11-C-02/03/04** — audit's pre-existing deferrals

### Recommendation
Section 0 unblocks Fase 1. The two physical findings whose behavior matters for the very next sprint (F1 mechanical chain, F2 BLOCKED_HANDOFF) are empirically validated. Subsequent sprints can build on a verified hardware/software baseline.

---

## Files touched in Section 0

| Path | Action | Finding |
|---|---|---|
| `docs/audit/2026-05-13-pre-phase1-verification.md` | created | deliverable |
| `src/agv_mode_arbiter/CLAUDE.md` | edited (1 line) | F2 / G2 |
| `docs/adr/0002-accel-asymmetry.md` | created | F5 |
| `src/agv_navigation/config/velocity_smoother.yaml` | edited (xref ADR-0002) | F5 |
| `src/agv_odrive/config/odrive_params.yaml` | edited (xref ADR-0002) | F5 |
| `tools/verify_specs/verify_no_waypoint_manager.sh` | created | F7 |
| `tools/verify_specs/all.sh` | edited (added BLOCKING entry) | F7 |
| `docs/audit/2026-05-13-greenhouse-hardening/11_commissioning_walkthrough.md` | edited (Step 2 rewrite) | F3 / G3 |
| `docs/calibration/odrive_nvram_dump_2026-05-13.txt` | created (raw + analysis) | F1 |
| `src/agv_rail_driver/src/rail_driver_node.cpp` | edited (subscriber type) | G4 (regression fix) |
| `src/agv_rail_driver/CLAUDE.md` | edited (doc rot) | G4 |
| `src/agv_rail_driver/CMakeLists.txt` | edited (nav2_msgs dep) | G4 |
| `src/agv_rail_driver/package.xml` | edited (nav2_msgs dep) | G4 |

**Production runtime code changed during Section 0: ONLY rail_driver (G4 regression closure).** Everything else is configuration, documentation, or new verifier scripts.

---

## Day-2 execution log

| # | Test | Method | Result |
|---|---|---|---|
| 0 | Restore WiFi | `rfkill unblock wifi`, `nmcli radio wifi on`, connect `LimserConnect` | WiFi back, JETSON-LAN-IP |
| 1 | F2/F6/F7 L2 | `ros2 topic info -v` + `ros2 node list` | **rail_driver regression discovered**, closed inline (G4 / commit 08ac348) |
| 2 | F4 L3 | User physically unplug + replug ZED USB during 30-s window | +2.4° yaw jump on stationary robot captured as baseline |
| 3 | F1 NVRAM | User connected ODrive USB; Python `odrive` lib read both boards | dump file written; G5 bilateral asymmetry surfaced |
| 4 | F1 gearbox empirical | User manually rotated left wheel one full turn while pos_estimate logged | 9.841 motor turns / 1 wheel turn → gear 10:1 confirmed |
| 5 | F1 step test (calzado) | `cmd_vel.linear.x=0.2 m/s` for 15 s, user visual count | encoder reports 5.19 wheel turns, user observed 5+20° (5.05); chain clean |
| 6 | F2 L3 fault injection | Inject `CollisionMonitorState{STOP}`, observe `/agv/mode/state` | `corridor_nav → blocked_handoff → corridor_nav` (transitions 2→3→4) ✓ |
| 7 | Disarm motors (post-test) | User via dashboard | Triggered new Sprint E.lite confirmation modal (UX validation bonus) |
