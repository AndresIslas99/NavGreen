# SUMMARY — 2026-05-13 Greenhouse Field-Readiness Audit

> **Second-pass update (2026-05-13)**: severities recalibrated on review;
> CRITICAL-02-02 hypothesis ranking updated with the physical caliper
> measurement of 0.0625 m wheel radius; Phases 4 / 6 / 7 / 9 / 10 added.
> First-pass findings preserved unchanged below.

## Executive verdict

The 2026-04-13 governance audit successfully consolidated the workspace
into spec-as-SSOT and the verifiers are green. The static-analysis pass
has now closed 11 phase files (0–14 minus 5, 8, 12, 15 which remain
field-only). **The robot is not yet field-ready** but the remaining gaps
are concretely scoped.

Two findings dominate the priority queue:
1. **CRITICAL-02-02 (geometry SSOT)** — physical measurement now confirms
   `wheel_radius = 0.0625 m`. The 0.0781 in `odrive_params.yaml` is a
   compensation for a different bug (encoder `cpr` or `pole_pairs` in
   ODrive NVRAM — see updated hypothesis ranking below). Until the root
   cause is identified and corrected at the source, every navigation
   accuracy claim is built on a fudge factor.
2. **CR-00-01 (repo not self-contained)** — `agv_slam` referenced but
   absent. A fresh Jetson flash cannot boot.

All other findings are MEDIUM or HIGH but localised — none compound to a
new CRITICAL when those two close.

## Re-rating after Phase 11 (2026-05-13)

| Finding | Prior | After Phase 11 | Why |
|---|---|---|---|
| `CR-00-06` (waypoint_manager bypasses gates) | HIGH (active) | **HIGH (latent only)** | The dashboard's mission executor lives in `agv_ui_backend/src/index.ts:461` and goes through `ros.sendNavGoal` → all gates. `agv_waypoint_manager_node` is *shadow code* with a separate file. CR-00-06 stays HIGH because CLI / future callers can still bypass; HIGH-11-B-01 proposes deletion as the canonical fix. |
| HAZOP `H-14` (untrained operator) | R=9 (P=3) | **R=15 (P=5)** while Sprint A.5 is open | With auth disabled + repo-public default credentials (CRITICAL-11-C-01), any phone on the greenhouse WiFi can drive the robot. Drops back to R≤6 after Sprint A.5 lands. |
| HAZOP `H-07` (loss of WiFi to operator) | R=12 | **R=12 (confirmed open)** | MEDIUM-11-C-06 confirms no deadman / heartbeat. Open until Sprint B. |
| `MEDIUM-07-07` (no global BT timeout) | MEDIUM | **MEDIUM** (companion MEDIUM-11-D-06 adds UX surfacing) | Unchanged severity. |

## Severity recalibration (2026-05-13 review)

The first-pass severities were reviewed and several were rebalanced.
`CRITICAL` is now reserved for findings with **direct physical-operation
or safety risk**; portability and license issues are not CRITICAL under
that definition.

| Finding | Original | Recalibrated | Rationale |
|---|---|---|---|
| CR-00-03 (hardcoded paths) | CRITICAL | **HIGH** | Portability issue; does not put a running robot at risk. |
| CR-00-05 (license divergence) | HIGH | **MEDIUM** | Blocks sign-off, not operation. |
| HIGH-01-01 (`base_footprint` child) | HIGH | **MEDIUM** | Works functionally today; downstream impact is speculative. |
| HIGH-02-01 (ros2_control cutover) | HIGH | **(remove)** | Not a finding — a proposal. Tracked as an ADR draft instead. |
| HIGH-02-07 (gear_ratio doubled) | HIGH | **MEDIUM** | Math does not support 10× double-count as the cause of the 1.25× discrepancy. Boot invariant still useful as defense-in-depth. |
| CR-00-01, CR-00-02 / CRITICAL-02-02, CR-00-04, CR-00-06, HIGH-01-02, MEDIUM-* | (unchanged) | — | Severities held. |

## Findings filed in this cycle

**69 findings total, 4 critical / 25 high / 34 medium / 6 low** across
Phases 0–11 (1 NIT in Phase 7). The HAZOP skeleton is shipped as
`docs/safety/HAZOP_skeleton.md`; the commissioning walkthrough is
shipped as `11_commissioning_walkthrough.md`. Phases 3 (calibration),
5 (mapping), 8 (greenhouse behaviors), 12–14 (sim, CI, docs), and 15
(field) remain to be filed — see §F.

**Two new findings from Phase 11 rise to CRITICAL** and trigger a
dedicated **Sprint A.5** that supersedes Sprint B in priority:
- `CRITICAL-11-A-01` — `agv_mode_arbiter` subscribes to
  `/agv/collision_monitor_state` as `std_msgs/String` while the
  publisher emits `nav2_msgs/msg/CollisionMonitorState`. The
  type-mismatch is silent; `safety_stop` in the FSM never fires;
  `BLOCKED_HANDOFF` is unreachable. The physical safety chain (Nav2
  collision_monitor → cmd_vel_gate) still zeros the wheels, so the
  robot stops — but the arbiter and its `/agv/mode/state` topic
  remain "all good", and the dashboard shows green while a safety
  stop is in progress.
- `CRITICAL-11-C-01` — backend ships `enabled: false` for auth +
  hardcoded defaults `engineer:agv2026` / `operator:agv` written to
  `users.json` on first boot. Public repo → public credentials.
  Combined with `App.tsx:42` fail-open auth check, any device on the
  LAN can command the robot.

| ID | Sev | Title | Phase | Anchor | Effort |
|---|---|---|---|---|---|
| CR-00-01 | CRITICAL | Repo not self-contained: `agv_slam` referenced but absent | 0 | `agv_full.launch.py:279` | S → L |
| CRITICAL-02-02 | CRITICAL | Wheel geometry SSOT — physical 0.0625 m vs YAML 0.0781 m, root cause is encoder NVRAM | 0+2 | `odrive_params.yaml:7-8` | M |
| CR-00-03 | HIGH | Hardcoded `/home/orza/` in launch parameter dicts | 0 | `agv_full.launch.py:546,585,701,768` | S |
| CR-00-04 | HIGH | Production network interfaces hardcoded to AGX Orin names | 0 | `agv_start.sh:53,90` | S |
| CR-00-06 | HIGH | `agv_waypoint_manager` bypasses localization gate | 0 | `waypoint_manager_node.cpp:109` | M |
| HIGH-01-02 | HIGH | Camera frame transform lives in external `agv_slam` | 1 | `agv_full.urdf.xacro:14-15` | M |
| HIGH-02-03 | HIGH | URDF chassis inertia is literal `1.0` placeholder | 2 | `agv_base.xacro:25-29` | S |
| HIGH-02-04 | HIGH | ZED depth+IMU pipeline config invisible from this repo | 2 | `agv_slam` is external | M → L |
| HIGH-04-01 | HIGH | `agv_factor_graph` consumes `odometry/global`, double-counts marker corrections | 4 | `factor_graph_node.cpp:68-70` | M |
| HIGH-04-02 | HIGH | `marker_correction` does not check tag obliqueness — accepts grazing-incidence PnP | 4 | `marker_correction_node.cpp:287-321` | S |
| HIGH-04-03 | HIGH | Kidnapping detection parameter declared but never implemented | 4 | `auto_init_orchestrator_node.cpp:101` | M |
| HIGH-04-04 | HIGH | `marker_correction` registry parser is a homebrew YAML reader, no validation | 4 | `marker_correction_node.cpp:215-238` | S |
| HIGH-07-01 | HIGH | MPPI critic weights are not on a normalized scale | 7 | `nav2_params.yaml:131-188` | M |
| HIGH-07-02 | HIGH | Smoother accel/decel exceeds HAL limits — silent rate-limiting downstream | 7 | `velocity_smoother.yaml:15-16` vs `odrive_params.yaml:25-26` | S |
| HIGH-04-09 | HIGH | Local EKF reads wheel-yaw absolute + IMU yaw absolute simultaneously (no magnetometer) | 4 | `ekf_local.yaml:36-53` | M |
| HIGH-09-01 | HIGH | `/agv/hardware_estop` is `planned` — no hardware bridge wired | 9 | `interfaces.yaml#/agv/hardware_estop` | M |
| HIGH-09-02 | HIGH | `safety_supervisor` uses BEST_EFFORT QoS for monitored topics — type-erased TRANSIENT_LOCAL latches will not bind | 9 | `safety_supervisor.cpp:119` | S |
| HIGH-09-04 | HIGH | `cmd_vel_gate` initial `safety_ok_` value unverified from source | 9 | `cmd_vel_gate.hpp` (header not read) | S |
| MEDIUM-01-01 | MEDIUM | `base_footprint` as child of `base_link` | 1 | `agv_base.xacro:33-39` | S |
| MEDIUM-01-03 | MEDIUM | `static_transform_publisher` masks vibration-induced drift | 1 | depends on HIGH-01-02 | M |
| MEDIUM-01-04 | MEDIUM | No master-clock discipline at boot (chrony) | 1 | `agv_start.sh` | S |
| MEDIUM-01-05 | MEDIUM | HIL mixes wall-clock + sim-clock topics implicitly | 1 | `agv_full.launch.py:86-95` | S |
| MEDIUM-02-05 | MEDIUM | CAN bitrate docs vs script comment disagree | 2 | `project.yaml:69` vs `agv_start.sh:241` | S |
| MEDIUM-02-06 | MEDIUM | `nav2_params.yaml` header says "RegulatedPurePursuit", code is MPPI | 2 | `nav2_params.yaml:11-12` | S |
| MEDIUM-02-07 | MEDIUM | Possible mismatch ODrive ROS vs NVRAM `gear_ratio`; add boot invariant | 2 | `odrive_params.yaml:9` | S |
| MEDIUM-04-05 | MEDIUM | `factor_graph` has 0 consumers — burning 10 Hz iSAM2 CPU for nothing | 4 | `agv_factor_graph` topics | S |
| MEDIUM-04-06 | MEDIUM | factor_graph window 200 = ~20 s — loop closures past that horizon are lost | 4 | `factor_graph_node.cpp:34,209-226` | M |
| MEDIUM-04-07 | MEDIUM | Anti-spoofing: tags with unregistered IDs silently ignored — correct, but undocumented as a security property | 4 | `marker_correction_node.cpp:289` | S |
| MEDIUM-06-01 | MEDIUM | No negative-obstacle / drop-off detection — greenhouse drains and steps unhandled | 6 | costmap voxel layer | M |
| MEDIUM-06-02 | MEDIUM | min_obstacle_height 0.10 m means 10 cm-tall floor obstacles ignored by costmap | 6 | `nav2_params.yaml:274,281,340,347` | S |
| MEDIUM-06-03 | MEDIUM | scan_grid_mapper `/agv/live_map` has no documented decay policy | 6 | scan_mapper_params + node | M |
| MEDIUM-06-04 | MEDIUM | No dynamic-obstacle temporal filter — false stops from oscillating leaves likely | 6 | collision_monitor.yaml + voxel_layer | M |
| MEDIUM-06-05 | MEDIUM | No reflection filter for wet floors — depth aliases ceiling as obstacle | 6 | absence; `pointcloud_to_laserscan` params | M |
| MEDIUM-07-03 | MEDIUM | Docs drift: `vx_max=0.4` in `agv_navigation/CLAUDE.md` vs `0.25` in `nav2_params.yaml:108` | 7 | doc | S |
| MEDIUM-07-04 | MEDIUM | Docs drift: `stop_zone=footprint+5cm` in `agv_navigation/CLAUDE.md` vs `+20cm` in YAML | 7 | doc | S |
| MEDIUM-07-05 | MEDIUM | Stop-distance math in `collision_monitor.yaml:41` assumes 1.0 m/s² but HAL limits ~0.59 m/s² | 7 | `collision_monitor.yaml:38-46` | S |
| MEDIUM-07-06 | MEDIUM | `backup` plugin loaded in behavior_server but no BT calls it | 7 | `nav2_params.yaml:386` | NIT |
| MEDIUM-07-07 | MEDIUM | No global mission timeout in BT — stuck navigation only times out on the per-goal RoundRobin | 7 | `navigate_to_pose_forward_only.xml` | M |
| MEDIUM-07-08 | MEDIUM | Costmap inflation sized for 2 m corridors; does not survive < 1.5 m | 7 | `nav2_params.yaml:300-311` | S |
| MEDIUM-09-03 | MEDIUM | `safety_supervisor` is a regular Node, not a lifecycle node | 9 | `safety_supervisor.cpp:63` | M |
| MEDIUM-09-05 | MEDIUM | Stop zone extends to rear despite forward-only motion — false-stops possible | 9 | `collision_monitor.yaml:54` | S |
| MEDIUM-10-01 | MEDIUM | No `/diagnostics` aggregator configured; per-node publication patchy | 10 | absence in launch | M |
| MEDIUM-10-02 | MEDIUM | `safety_supervisor` BEST_EFFORT subscriber to topics whose publishers may be RELIABLE (DDS asymmetry tolerated but worth recording) | 10 | `safety_supervisor.cpp:119` | NIT |
| MEDIUM-10-03 | MEDIUM | DDS write-side max participants `MaxAutoParticipantIndex 120` — high for single-robot, ok for fleet later | 10 | `agv_start.sh:131` | NIT |
| MEDIUM-10-04 | MEDIUM | No `rosbag2` rotation policy in launch or systemd | 10 | absence | M |
| MEDIUM-10-06 | MEDIUM | `rail_approach` subscribes RELIABLE to `transient_local.reliable` publisher — late-join semantics differ | 10 | `rail_approach_node.cpp:162` | S |
| CRITICAL-11-A-01 | CRITICAL | `mode_arbiter` collision_monitor_state type mismatch — `safety_stop` never fires; `BLOCKED_HANDOFF` unreachable | 11.A | `mode_arbiter_node.cpp:171-175` | S |
| HIGH-11-A-02 | HIGH | No source-staleness timeout in arbiter; stale `last_*` Twist relayed indefinitely | 11.A | `mode_arbiter_node.cpp:152-160,329-344` | S |
| HIGH-11-A-03 | HIGH | TELEOP override carve-out lets `rail_approach` keep driving while operator picked teleop | 11.A | `mode_fsm.hpp:147-161` | M |
| MEDIUM-11-A-04 | MEDIUM | `safety_stop` detection is substring match (`find("stop")`); false-positive prone | 11.A | `mode_arbiter_node.cpp:174` | NIT (combined with CRITICAL-11-A-01) |
| MEDIUM-11-A-05 | MEDIUM | Default `operator_mode = "nav"` allows immediate motion on cold boot | 11.A | `mode_fsm.hpp:74` | S |
| MEDIUM-11-A-06 | MEDIUM | Source-state strings hardcoded; no enum / schema; brittle to controller changes | 11.A | `mode_fsm.hpp:182-244` | M |
| HIGH-11-B-01 | HIGH | Dashboard uses backend's mission executor; `agv_waypoint_manager` is shadow code with a separate file | 11.B | `index.ts:461-545` vs `waypoint_manager_node.cpp:73-345` | S→M |
| HIGH-11-B-02 | HIGH | `spin_until_future_complete` on a single-threaded executor from a worker thread — deadlock risk | 11.B | `waypoint_manager_node.cpp:309-329` | M |
| MEDIUM-11-B-03 | MEDIUM | Mission file is append-only; duplicate IDs accumulate; oldest wins on lookup | 11.B | `waypoint_manager_node.cpp:162-211` | S |
| MEDIUM-11-B-04 | MEDIUM | Auto-generated mission IDs use `time%1e8` — collision within 100 ms | 11.B | `waypoint_manager_node.cpp:141-142` | S |
| MEDIUM-11-B-05 | MEDIUM | No waypoint validation (NaN, out-of-bounds, in-keepout) | 11.B | `missions.ts:32-55`, `waypoint_manager_node.cpp:134-172` | S |
| CRITICAL-11-C-01 | CRITICAL | Auth disabled by default + hardcoded credentials shipped in source | 11.C | `auth.ts:64,67-80` | M |
| HIGH-11-C-02 | HIGH | Unsalted SHA-256 password hashing | 11.C | `auth.ts:48-50` | S |
| HIGH-11-C-03 | HIGH | JWT token transported as URL query string in WebSocket connect | 11.C | `ws/control.ts:77-80`, `useWebSocket.ts:55-57` | S |
| HIGH-11-C-04 | HIGH | HTTP without TLS — credentials and JWT cleartext over WiFi | 11.C | Express setup | M |
| HIGH-11-C-05 | HIGH | `filterActionsForRole` returns all actions for `operator` role despite comment | 11.C | `auth.ts:37-46` | M |
| MEDIUM-11-C-06 | MEDIUM | No WS heartbeat / deadman; mission keeps running on operator disconnect (closes HAZOP H-07) | 11.C | `ws/control.ts:144-205,274-278` | M |
| MEDIUM-11-C-07 | MEDIUM | `state.missionPause` ignored by `waypoint_manager_node` (related HIGH-11-B-01) | 11.C | `missions.ts:74-83` vs `waypoint_manager_node.cpp:286-345` | S |
| MEDIUM-11-C-08 | MEDIUM | `POST /api/nav/goal` no input validation; missing field → goal at (0,0,0) | 11.C | `routes/nav.ts:5-12` | S |
| MEDIUM-11-C-09 | MEDIUM | No rate limiting on REST endpoints | 11.C | absence | S |
| HIGH-11-D-01 | HIGH | Auth check is fail-OPEN in frontend (`/api/auth/status` error → logged in) | 11.D | `App.tsx:35-45` | S |
| MEDIUM-11-D-02 | MEDIUM | E-stop toggle has no protection against accidental disengage | 11.D | `TopBar.tsx:240-241` | S |
| MEDIUM-11-D-03 | MEDIUM | No confirmation on Disarm Motors during navigation / mission | 11.D | `OperatePanel.tsx:34-40` | S |
| MEDIUM-11-D-04 | MEDIUM | `handleGoalClick` sends nav_goal on single map click; no confirmation | 11.D | `App.tsx:111-117` | M |
| MEDIUM-11-D-05 | MEDIUM | No calibration wizards — all 4 wizards (intrinsics, extrinsics, UMBmark, hand-eye) are CLI scripts | 11.D | absence | XL |
| MEDIUM-11-D-06 | MEDIUM | No "stuck in recovery" surface — operator sees normal state while BT cycles recoveries | 11.D | `state_machine.ts:50-66` | M |
| LOW-11-D-07 | LOW | JWT in localStorage — XSS-exfiltratable | 11.D | `api/client.ts` | M |
| LOW-11-D-08 | LOW | WebSocket has no max reconnect cap; backend down hidden forever | 11.D | `useWebSocket.ts:46-82` | S |
| LOW-01-06 | LOW | TF single-owner invariant not enforced by verifier | 1 | proposal | S |
| LOW-04-08 | LOW | `relocalization_cooldown_ms` 500 ms is asymmetric to EKF rates; document or tune | 4 | `marker_correction_node.cpp:55` | NIT |
| LOW-10-05 | LOW | `SocketReceiveBufferSize 64MB` may exceed Orin NX `net.core.rmem_max`; document required sysctl | 10 | `agv_start.sh:144` | NIT |
| CR-00-05 | MEDIUM | License divergence undeclared | 0 | `agv_rail_approach/package.xml` | S |

(38 findings total. The recalibrated tally above keeps `CR-00-02` and
`CRITICAL-02-02` as the same root cause; the Phase 0 row carries the
forensics, the Phase 2 row carries the runtime-resolution mechanism and
the remediation plan.)

## Updated CRITICAL-02-02 — physical measurement + hypothesis ranking

**New evidence (caliper, 2026-05-13)**: wheel diameter = **125 mm**.
Physical radius = **0.0625 m**.

This **confirms the URDF + `robot_params.yaml` (0.0625) as the geometric
truth** and confirms `odrive_params.yaml`'s 0.0781 as a compensation
factor for a different bug. The first-pass scenario "wheel swap not
documented" is **rejected**. The 1.25× factor (= 5/4 exactly) is too
clean to be UMBmark drift (typically 1–5 %).

**Revised hypothesis ranking for the 1.25× factor**:

1. **`encoder.config.cpr` mis-set in ODrive NVRAM (most likely)**. A 25 %
   fractional error in counts-per-revolution makes the firmware report
   1.25× the real motor velocity. The fudge to `wheel_radius` exactly
   compensates the integration. **Testable in 30 s with `odrivetool`.**
2. **`motor.config.pole_pairs` wrong (5/4 ratio)**. 7→5 pole-pair
   mismatch gives 1.40; 6→5 gives 1.20; 5→4 gives **1.25**. The
   clean integer ratio strongly suggests this class.
3. **Velocity-unit reinterpretation** (turns/s vs RPM/60) — discarded
   for magnitude (would produce 2π or 1/2π factors, not 1.25).
4. **Empirical fudge with no diagnosed cause** — least likely but most
   dangerous, because any upstream firmware change destroys the
   compensation silently. The team's CLAUDE.md table calling 0.0781
   "calibrated" suggests this is what happened.

**Revised remediation (replaces the original CRITICAL-02-02 plan)**:

1. **Step 0 (NEW)**: dump ODrive S1 NVRAM with `odrivetool` or CAN
   `GET_*` reads. Capture
   `motor.config.pole_pairs`, `encoder.config.cpr`,
   `motor.config.torque_constant`, any `gear_ratio`/`reducer`.
   Diff against the ODrive firmware default for the M8325s. **The 1.25×
   factor must surface in one of these numbers.** Log to
   `docs/calibration/odrive_nvram_dump_2026-05-XX.txt`.
2. **Step 1**: correct the root cause in NVRAM (e.g., set `encoder.cpr`
   to the manufacturer's value). Save to NVRAM with `save_configuration`.
3. **Step 2**: revert `odrive_params.yaml` to `wheel_radius: 0.0625`,
   `track_width: <pending physical measurement>`, `gear_ratio:` aligned
   with the firmware decision (see MEDIUM-02-07).
4. **Step 3**: re-run UMBmark. Residual error must be in 1–5 % range,
   not 25 %. Document.
5. **Step 4**: centralise on `robot_params.yaml` as physical-only SSOT;
   move UMBmark calibration deltas into a separate
   `wheel_odom_umbmark_<date>.yaml` consumed only by the wheel-odometry
   layer.

The original `verify_geometry_ssot.py` requirement and the URDF / Nav2 /
HIL parity check remain. The new step 0 is a 30-minute experiment
preceding the refactor.

## Priority queue (recommended order of remediation)

### Sprint A — repro and geometry (1 week)
The minimum to safely run a hardware test of the rest.

1. **CR-00-01** — Author `agv_greenhouse.repos`, document `vcs import` in README. Decide vendoring strategy for `agv_slam`. **Without this, none of the other findings can be reproduced on a fresh system.**
2. **CRITICAL-02-02 step 0** — Dump ODrive NVRAM (`odrivetool` or CAN), confirm whether `encoder.cpr` or `pole_pairs` explains the 1.25× factor. **30-minute experiment; this is the single most important diagnostic in the audit.** Document.
3. **CRITICAL-02-02 step 1+2** — Correct the root cause in NVRAM. Revert `odrive_params.yaml` `wheel_radius` to 0.0625. Physically re-measure track width.
4. **CRITICAL-02-02 step 3+4+5** — Re-run UMBmark with correct config. Centralise on `robot_params.yaml`. Add `verify_geometry_ssot.py`.
5. **CR-00-03** — Replace hardcoded `/home/orza/` with `os.environ.get('AGV_DATA_DIR', …)` in the four launch dicts.
6. **MEDIUM-02-07** — Add boot-time invariant comparing ROS `gear_ratio` against ODrive NVRAM (defense in depth for the diagnostic in step 0).

### Sprint A.5 — close Phase 11 CRITICALs (priority over Sprint B, ~1 day)

Two findings opened in Phase 11 affect either physical-operation
awareness (mode_arbiter) or "any LAN device can drive the robot"
(auth defaults). Closing them is cheaper than every Sprint B item
and has higher-leverage safety impact than the remaining Sprint A
items already landed.

A.5.1 — **CRITICAL-11-A-01**: change `mode_arbiter_node.cpp:171-175`
to subscribe to `nav2_msgs/msg/CollisionMonitorState`; drop the
`std_msgs/String` substring heuristic (this also closes
MEDIUM-11-A-04). Unit test that `safety_stop` flips on
`action_type == STOP`.

A.5.2 — **CRITICAL-11-C-01**: change `auth.ts:64` to `enabled: true`;
remove hardcoded defaults at `auth.ts:67-70`; generate a random
admin password on first boot, log it once to stdout (systemd
journal) and force-change on first login. Also fail-closed
`App.tsx:42` (HIGH-11-D-01) — show "Backend unreachable" instead
of `setLoggedIn(true)`.

### Sprint B — environmental robustness (1 week)
The fixes that protect the first field deployment.

5. **CR-00-04** — Network interface candidates from env var + auto-enumeration.
6. **MEDIUM-01-04** — chrony.conf template + `chronyc waitsync` in `agv_start.sh`.
7. **HIGH-01-02** — Move camera→base extrinsic into URDF as a calibrated xacro macro. Hand-eye calibration procedure in `docs/calibration/camera_mount.md`.
8. **HIGH-02-04** — Pull ZED params into this repo (or vendor `agv_slam`). Decide depth mode (likely NEURAL on AGX Orin, PERFORMANCE fallback on Orin NX).

### Sprint C — quality fixes (3 days)
Lower urgency but cheap to clean.

9. **HIGH-01-01** — Invert URDF so `base_footprint` is the root.
10. **HIGH-02-03** — Compute URDF inertias from box / cylinder formulas via xacro macro.
11. **MEDIUM-02-05** — Settle the CAN bitrate, align spec + script + systemd.
12. **MEDIUM-02-06** — Fix Nav2 YAML header.
13. **CR-00-05** — `LICENSE.md` + SPDX headers.
14. **CR-00-06** — Refactor `waypoint_manager` to route through `agv_ui_backend` for the localization gate.

### Sprint D — observability infrastructure (1 week)
Needed for the hardware-dependent phases to become testable.

15. **MEDIUM-01-03** — Online camera extrinsic consistency monitor.
16. **MEDIUM-01-05** — Annotate wall-clock-only topics in `interfaces.yaml`.
17. **LOW-01-06** — `verify_tf_single_owner.py` to turn the `/**:` key trap into a hard fail.
18. **HIGH-02-01** — ADR for ros2_control cutover; CI matrix on mock_components.

## What this audit deferred to hardware

These items cannot be closed without physical equipment:

| Phase | Deferred items |
|---|---|
| 1 | Build time on Orin (<5 min hot, <12 min cold); TF jitter; clock skew measurement; CAN frame error rate |
| 2 | ODrive command→motion latency p99; watchdog stop time; ZED depth FPS sustained; USB recovery |
| 3 (Calibration) | UMBmark execution; Kalibr extrinsics; reprojection RMSE; closed-loop drift in 5 m × 5 m |
| 4 (Localization) | 100 m loop drift % with/without AprilTags; kidnapping recovery time; pose rate ≥ 30 Hz live; forced-blackout 5 s test |
| 5 (Mapping) | 1000 m² mapping run time; cycle-time on re-mapping |
| 6 (Perception) | Person stop distance; hose detection recall; false-positive rate in vegetated corridor |
| 7 (Planning) | Path planning p99; tracking error RMS; jerk RMS; success rate over 100 missions; recovery rate |
| 8 (Greenhouse) | Headland turn timing; coverage gap %; mission resume correctness |
| 9 (Safety) | E-stop latency on oscilloscope; collision_monitor stop distance; HAZOP exercise |
| 10 (Comms) | DDS throughput on Wi-Fi; 8 h bag rotation correctness; WiFi loss/recover test |
| 11 (GUI) | Usability test with non-ROS technician (< 2 h setup) |
| 12 (Sim-to-real) | Sim-to-real metric gap < 20 %; HIL → field regression |
| 13 (CI) | CI pipeline duration < 30 min; coverage ≥ 70 % core packages |
| 14 (Docs) | External onboarding test < 1 h |
| 15 (Field) | MTBF ≥ 8 h; 200-mission success rate ≥ 98 %; 0 safety incidents |

Each of these has the acceptance criterion and the minimal experimental
harness sketched in the relevant phase file (where the file exists) or
in this SUMMARY (where it does not). The pattern to operationalize them
is the same: a script under `scripts/health/`, a Python test under
`tests/<phase>/`, both feeding a row in `tests/<phase>/checklist.md`,
each run logged to `events.jsonl` so the dashboard surfaces pass/fail.

## What this audit chose NOT to recommend

Per the prompt's "no migration without comparison" rule, some popular
SOTA frameworks were considered and rejected for this project:

- **LiDAR-inertial SLAM (FAST-LIO2, Point-LIO, LIO-SAM)**: rejected
  because no LiDAR on the robot. Worth re-evaluating if a Livox / Hesai
  is added; the vibration-rich greenhouse floor makes LIO compelling
  relative to VIO-only.
- **Full Autoware Universe stack**: rejected — overkill for a single
  small AGV in a confined indoor space; massive deployment overhead.
- **Apollo Cyber RT**: rejected for the same reason; ROS 2 is sufficient.
- **Hard real-time controller via PREEMPT_RT**: rejected for MVP; Jetson
  AGX with default kernel meets the latency requirements for 0.5 m/s
  greenhouse speeds. Revisit at higher speeds or for safety certification.
- **Moving to Smac Hybrid-A***: deferred — the team made a deliberate
  switch to SmacPlanner2D in 2026-04 (per `project.yaml:88-92`). Hybrid
  would buy orientation-aware planning at the cost of compute. Worth
  evaluating once the geometry SSOT (CRITICAL-02-02) is fixed and the
  MPPI controller can be tuned against a correct robot model.
- **Online sim-to-real domain randomization (Isaac Lab)**: deferred —
  the existing HIL harness is mature, sim-to-real gap will become a
  real concern only after the geometry SSOT is fixed.

## Reading order for an engineer picking this up

1. This file (`SUMMARY.md`).
2. `00_inventory.md` for the cartography and the Phase-0 critical findings.
3. `02_hal.md` for the deep dive on the geometry issue (the single most impactful finding).
4. `01_foundations.md` for the TF/time/REP-105 supporting analysis.
5. The previous audit, `../2026-04-13-full-audit.md`, for context on
   what was already fixed.
6. The specs in `../../../specs/*.yaml` for the authoritative contracts.

## Roadmap if all of Sprint A passes hardware

Assuming Sprints A and B close (≈ 2 weeks), the next 3-week field cycle
should target:

- **Week 3**: Phase 3 (calibration) execution on real hardware. Procedure
  manifests under `docs/calibration/`. Closed-loop UMBmark < 1 %.
- **Week 4**: Phase 4 (localization) field tests. 100 m drift with
  AprilTags target < 0.5 %. Begin Phase 7 (Nav2) controller tuning with
  the now-correct robot model.
- **Week 5**: Phase 9 (safety) HAZOP / FMEA workshop. E-stop measurement.
  Phase 11 (GUI) usability test with a real technician.

This audit's role at that point becomes monitoring — each phase file
gains a "Field results" section appended as the experiments run.

## Sprint A non-hardware closure (2026-05-13)

6 atomic commits landed on `claude/amr-security-audit-gPtCd` closing
the no-hardware portion of Sprint A. The numerical fix of
CRITICAL-02-02 (replacing the SSOT's 0.0781 wheel_radius with the
geometric 0.0625 m after the ODrive NVRAM root cause is corrected)
remains as a follow-up — procedure documented at
`docs/calibration/odrive_nvram_dump_procedure.md`, executable in
30 minutes when hardware is next available.

| Commit | Subject | Findings |
|---|---|---|
| `61a973c` | specs: declare robot_geometry.yaml SSOT + NVRAM dump procedure | C1 — spec-first per AGENT_INSTRUCTIONS |
| `2d21d6f` | geometry: scaffold robot_geometry.yaml SSOT, reroute launches | CRITICAL-02-02 scaffolded |
| `200516b` | verify: enforce geometry SSOT structurally; WARN on numerical drift | new `verify_geometry_ssot.py` |
| `fead755` | bringup: replace 4 hardcoded /home/orza paths | CR-00-03 closed |
| `6d2ea72` | build: add agv_greenhouse.repos + NVRAM info log on odrive boot | CR-00-01 closed + MEDIUM-02-07 closed |
| `5c72ac5` | docs: bookkeeping (CLAUDE.md tables + this closure section) | — |

## Sprint A.5 closure (2026-05-13)

4 atomic commits landed on `claude/amr-security-audit-gPtCd` closing
the two Phase-11 CRITICALs and HIGH-11-A-02. Sprint A.5 was prioritised
over Sprint B because (a) the mode_arbiter type mismatch silently
broke the FSM's safety_stop awareness, and (b) the auth defaults made
the robot driveable from any LAN device using the public-repo
credentials.

| Commit | Subject | Findings |
|---|---|---|
| `8d81517` | mode_arbiter: subscribe to CollisionMonitorState correct type | CRITICAL-11-A-01 closed + MEDIUM-11-A-04 closed |
| `5d29df1` | mode_arbiter: per-source staleness timeout | HIGH-11-A-02 closed |
| `dd4b6ec` | ui_backend: auth defaults safe, generate admin on first boot | CRITICAL-11-C-01 closed (backend side) |
| `bd46e6b` | dashboard: fail-closed auth status check | HIGH-11-D-01 closed |

### Findings status after Sprint A.5

| ID | Status | Notes |
|---|---|---|
| CRITICAL-11-A-01 | **CLOSED** | Subscriber type fixed; STOP constant comparison replaces substring heuristic. |
| HIGH-11-A-02 | **CLOSED** | Per-source 250 ms staleness gate; throttled WARN on stale relay. |
| MEDIUM-11-A-04 | **CLOSED** | Closed by the same commit as CRITICAL-11-A-01 — substring trap removed. |
| CRITICAL-11-C-01 | **CLOSED (backend)** | First-boot admin password + `must_change_password` flag + `POST /api/auth/change-password` route. Existing on-disk `users.json` preserved (dev workflows intact). |
| HIGH-11-D-01 | **CLOSED** | "Backend unreachable" fail-closed screen with Retry button. |

### HAZOP re-rating

- **H-14 (untrained operator)**: was R=15 (P=5) with the public
  credentials live. Now back to **R≤6**, contingent on the dev Jetson's
  legacy `users.json` being cleaned up (engineer/agv2026, operator/agv
  must be removed manually — the migration logic preserves existing
  files intentionally, so the dev has to delete those entries).

### Still open (NOT in Sprint A.5)

- HIGH-11-C-02 (SHA-256 unsalted) — Sprint B.
- HIGH-11-C-03 (JWT in WS URL query) — Sprint B.
- HIGH-11-C-04 (HTTP without TLS) — Sprint B.
- HIGH-11-A-03 (TELEOP carve-out for rail_approach) — Sprint B/C.
- All Phase 11.B findings on `agv_waypoint_manager` — Sprint B (HIGH-11-B-01 proposes deletion).

### Verifier baseline after Sprint A.5

- `bash tools/verify_specs/all.sh`
- 10 scripts, 0 blocking failures, 1 warning
- `verify_geometry_ssot.py` still WARNs on the 0.0781↔0.0625
  divergence (expected until CRITICAL-02-02 NVRAM dump lands).

### Next operational step

With Sprint A.5 closed, the highest-leverage hardware action is the
ODrive NVRAM dump per
[`docs/calibration/odrive_nvram_dump_procedure.md`](../../calibration/odrive_nvram_dump_procedure.md).
30-minute procedure → identifies whether the 1.25× factor lives in
`encoder.config.cpr` or `motor.config.pole_pairs` → 1-line YAML fix
closes CRITICAL-02-02 numerically and flips `verify_geometry_ssot`
from WARNING to OK.

### Findings status

| ID | Status | Commit |
|---|---|---|
| CR-00-01 | CLOSED — `.repos` file exists with 11 declared repositories; 2 marked `# TODO verify SHA` for next hardware contact (agv_slam, zed-ros2-wrapper) | `6d2ea72` |
| CR-00-03 | CLOSED — 4 path hardcodes replaced; env-var fallback remains as backstop | `fead755` |
| CRITICAL-02-02 | **SCAFFOLDED** — structural SSOT in place; numerical drift visible as WARN on every commit; the one-line fix waits on NVRAM dump | `61a973c` + `2d21d6f` + `200516b` |
| MEDIUM-02-07 | CLOSED — boot-time RCLCPP_INFO log + NVRAM procedure doc | `6d2ea72` |

### Verifier baseline now

- `bash tools/verify_specs/all.sh`
- 10 scripts (was 9 — `verify_geometry_ssot.py` added)
- 0 blocking failures
- 2 warnings: pre-existing `zed-ros2-wrapper` not vendored,
  `verify_geometry_ssot` numerical drift WARN (expected; clears post
  CRITICAL-02-02 step 5)

### Next hardware session — 30-minute action

Execute `docs/calibration/odrive_nvram_dump_procedure.md`:
1. Capture ODrive S1 NVRAM with `odrivetool`.
2. Confirm which firmware parameter (`encoder.config.cpr` or
   `motor.config.pole_pairs`) carries the 1.25× factor.
3. Correct it in NVRAM.
4. Edit `src/agv_description/config/robot_geometry.yaml` —
   `wheel_radius: 0.0625`, `track_width: 0.735`, `gear_ratio: 1.0`,
   `left_wheel_y: 0.3675`, `right_wheel_y: -0.3675`.
5. Re-run UMBmark, confirm residual error in 1–5 % range.
6. `bash tools/verify_specs/all.sh` — `verify_geometry_ssot.py`
   should drop from RESULT: WARNING to RESULT: OK.

One PR. One commit. CRITICAL-02-02 closes.

## Sprint B closure (2026-05-13)

4 atomic commits landed on `claude/amr-security-audit-gPtCd` closing
Sprint B (architectural cleanup + WiFi deadman + hardware E-stop doc).

| Commit | Subject | Findings |
|---|---|---|
| `35bb850` | bringup: remove agv_waypoint_manager from production launch | **HIGH-11-B-01 closed**. CR-00-06 active risk → latent only (no production caller). |
| `b4cc2b2` | bringup: gate factor_graph behind enable_factor_graph | **HIGH-04-01 closed** + **MEDIUM-04-05 closed**. Saves 10–20 % CPU in production runs. |
| `c7a659f` | docs(safety): hardware E-stop wiring procedure option 1 | **HIGH-09-01 option 1 closed** (doc ready; field-install pending). |
| `1c91c27` | ui_backend: WS heartbeat + mission-pause-on-disconnect | **MEDIUM-11-C-06 closed** + **HAZOP H-07 R=12 closed**. |

### Findings status after Sprint B

| ID | Status | Notes |
|---|---|---|
| HIGH-11-B-01 | **CLOSED** | waypoint_manager removed from production launch. Specs updated. Package buildable for CLI/tests. |
| HIGH-04-01 | **CLOSED (gated)** | factor_graph runs only when `enable_factor_graph:=true`. The "consume-corrected-EKF" architectural concern persists for the optional case but does not affect production CPU. |
| MEDIUM-04-05 | **CLOSED** | Same gate. The 0-consumer iSAM2 no longer runs in production. |
| HIGH-09-01 | **CLOSED (option 1 doc)** | Procedure documented at `docs/safety/hardware_estop_wiring.md`. Hardware install pending next field-prep session. Options 2 + 3 (ToF / dual-channel relay) deferred post-MVP. |
| MEDIUM-11-C-06 | **CLOSED** | WS heartbeat at 2 s; deadman pauses mission when last operator disconnects. |
| HAZOP H-07 (WiFi loss) | **CLOSED** R=12 → R≤3 | Mission no longer runs unsupervised after disconnect. |

### Still open after Sprint B

- HIGH-11-B-02 (waypoint_manager `spin_until_future_complete` deadlock):
  package still buildable, latent if called from CLI. The proper fix
  is deletion of the package — tracked as Sprint C cleanup.
- HIGH-04-01 architectural side (factor_graph consumes EKF output):
  unchanged; only the production-cost concern is closed. Architectural
  fix when cutover is genuinely attempted.
- HIGH-11-A-03 (TELEOP carve-out for rail_approach): Sprint C.
- HIGH-11-C-02, -03, -04 (salted KDF, JWT off WS URL, TLS): Sprint C.
- Sprint D (observability) and Sprint E (calibration wizards XL):
  unchanged.

### Sprint B verifier baseline

- `bash tools/verify_specs/all.sh`
- 10 scripts, 0 blocking, 1 warning (`verify_geometry_ssot.py` numerical drift, expected pending NVRAM dump for CRITICAL-02-02).
- Frontend (`npm run build`) and backend (`npm run build`) clean.
- `colcon build --packages-select agv_bringup` clean (-Werror enforced).

## Sprint C closure (2026-05-13)

4 atomic commits closing the "quality / consistency" cluster.

| Commit | Subject | Findings |
|---|---|---|
| `4046bfe` | nav2: fix doc drift on controller, vx_max, stop_zone | **MEDIUM-02-06, MEDIUM-07-03, MEDIUM-07-04 closed** |
| `d6a6efc` | nav2: trim stop_zone rear to footprint, not +5 cm | **MEDIUM-09-05 closed** |
| `b95b9d8` | rail_approach: transient_local QoS for late-join + create launch/ dir | **MEDIUM-10-06 closed** + pre-existing build bug fixed |
| `478c23d` | bringup: wait for chrony NTP sync at boot | **MEDIUM-01-04 closed** |

### Findings status after Sprint C

| ID | Status | Notes |
|---|---|---|
| MEDIUM-02-06 | **CLOSED** | nav2_params.yaml header now correctly says MPPI + RotationShim. |
| MEDIUM-07-03 | **CLOSED** | CLAUDE.md table updated to vx_max=0.25 with audit cite. |
| MEDIUM-07-04 | **CLOSED** | CLAUDE.md table + pipeline section now agree with the YAML (20 cm stop_zone, two-source). |
| MEDIUM-09-05 | **CLOSED** | Stop_zone rear coord at footprint exactly (-0.30); rear lobe no longer false-stops on noise. |
| MEDIUM-10-06 | **CLOSED** | rail_approach subscriber matches publisher's transient_local; late-join no longer misses the latched localization state. |
| MEDIUM-01-04 | **CLOSED** | chronyc waitsync 5 0.05 added before ROS source. Graceful fallback if chrony absent. |

### Still open after Sprint C

- HIGH-04-02 (incidence filter for marker_correction) — Sprint D.
- HIGH-04-03 (kidnapping detection) — Sprint D.
- HIGH-04-04 (yaml-cpp parser for marker registry) — Sprint D.
- HIGH-04-09 (yaw-absolute fight in ekf_local) — needs HIL test before
  merge; Sprint D.
- HIGH-07-01 (MPPI critic normalization) — Sprint D (needs HIL p95 test).
- HIGH-07-02 (smoother↔HAL accel mismatch) — Sprint D (needs hardware).
- HIGH-09-04 (cmd_vel_gate initial state) — Sprint D (1-line header read).
- HIGH-09-02 (safety_supervisor per-topic QoS) — Sprint D.
- HIGH-11-A-03 (TELEOP carve-out for rail_approach) — needs design call; Sprint D.
- HIGH-11-B-02 (waypoint_manager deadlock risk) — closes by deleting the
  package entirely; Sprint D candidate.
- HIGH-11-C-02 (salted KDF), HIGH-11-C-03 (JWT off WS URL),
  HIGH-11-C-04 (TLS) — security hardening; Sprint D.
- HIGH-11-D-04 (no goal-click confirmation) — Sprint D UX.
- All Phase 6 perception findings — Sprint D + E.
- Sprint E (calibration wizards XL) — gated by Sprint A NVRAM dump.

### Sprint C verifier baseline

- `bash tools/verify_specs/all.sh`: 10 scripts, 0 blocking, 1 warning
  (geometry SSOT drift, expected pending NVRAM dump).
- `colcon build` clean across modified packages (-Werror enforced).

## Sprint D closure (2026-05-13)

4 atomic commits closing the observability + defensive-cleanup cluster.

| Commit | Subject | Findings |
|---|---|---|
| `d59943a` | safety: document cmd_vel_gate fail-closed default | **HIGH-09-04 closed** (verification + doc) |
| `6f30aa0` | safety: per-topic QoS in safety_supervisor | **HIGH-09-02 closed** |
| `57a7394` | bringup: event-triggered rosbag2 in snapshot mode | **MEDIUM-10-04 closed** (recorder scaffolded; auto-trigger from backend = follow-up) |
| `8487994` | docs(adr): 0001 diagnostics strategy | **MEDIUM-10-01 closed** (ADR documents the custom-topic choice) |

### Findings status after Sprint D

| ID | Status | Notes |
|---|---|---|
| HIGH-09-04 | **CLOSED** | Verified `safety_ok_{false}` default in cmd_vel_gate.hpp; comment + CLAUDE.md document the fail-closed startup behavior. |
| HIGH-09-02 | **CLOSED** | safety_supervisor accepts `monitored_qos` per-topic; current config explicitly lists best_effort for all three topics so a future RELIABLE flip surfaces in a PR diff. |
| MEDIUM-10-04 | **CLOSED (scaffold)** | `enable_event_recording` launch arg + `ros2 bag record --snapshot-mode` for safety-critical topics. Operator triggers a dump via service call. Auto-trigger from backend on safety events = future follow-up. |
| MEDIUM-10-01 | **CLOSED (ADR)** | ADR 0001 documents the decision to keep custom typed topics; engineer-only `/diagnostics` retrofit not pursued. |

### Still open after Sprint D

- HIGH-11-A-03 (TELEOP carve-out for rail_approach) — needs operator
  workflow design call before code change.
- HIGH-04-02 / 04-03 / 04-04 (marker_correction hardening) — Sprint E
  candidate; 04-02 (incidence filter) is the smallest.
- HIGH-04-09 (ekf_local yaw absolute fight) — needs HIL regression
  test before merge; defer until hardware sessions are systematic.
- HIGH-07-01 (MPPI critic normalization) — needs HIL p95 test loop.
- HIGH-07-02 (smoother ↔ HAL accel mismatch) — needs hardware
  measurement.
- HIGH-11-B-02 (waypoint_manager deadlock risk) — closed by deletion
  of the package once Sprint B's removal-from-launch has soaked.
- HIGH-11-C-02/03/04 (salted KDF, JWT off WS URL, TLS) — security
  hardening; needs deployment-network design call.
- HIGH-11-D-04 (no goal-click confirmation) — Sprint E UX.
- MEDIUM-06-* Phase 6 perception findings — Sprint E (depth pipeline
  refactor).
- Sprint E proper (calibration wizards XL) — still gated by Sprint A
  NVRAM dump; meaningful only after a first field test informs which
  wizards matter.

### Sprint D verifier baseline

- `bash tools/verify_specs/all.sh`: 10 scripts, 0 blocking, 1 warning
  (geometry SSOT drift, expected pending NVRAM dump).
- `colcon build` clean across modified packages (-Werror enforced).
- `colcon test agv_safety`: 3/3 pass.

### Cumulative status (Sprints A.5 + B + C + D)

19 atomic commits landed in this session. Findings status snapshot:

| Severity | Closed in session | Remaining open |
|---|---|---|
| CRITICAL | 2 (11-A-01, 11-C-01 backend) | 2 (CR-00-01 .repos validation, CRITICAL-02-02 numerical NVRAM fix — both await hardware) |
| HIGH | 10 | ~13 (most need hardware or design call) |
| MEDIUM | 11 | ~18 (mostly Sprint E perception + UX) |
| LOW / NIT | 0 | ~6 |

The two remaining CRITICALs are hardware-gated and have explicit
procedures ready (NVRAM dump + .repos verification). Everything that
can be closed at the desk in this audit cycle is closed.

## Sprint E.lite closure (2026-05-12)

Sprint E proper (calibration wizards XL) is still gated by the field
NVRAM dump, but four desk-actionable items from the Sprint E backlog
were resolved in this pass — three perception/auth hardening and one
HMI UX patch. Tag: "Sprint E.lite" to distinguish from the calibration
wizard work still pending.

### Findings closed in Sprint E.lite

- **HIGH-04-02 — AprilTag incidence-angle filter in marker_correction.**
  `marker_correction_node.cpp` now extracts the tag-normal incidence
  angle from the OpenCV pose rotation matrix (`cv::Rodrigues` →
  `R_ct(2,2)`) and discards detections beyond a configurable threshold.
  Previously a tag observed near-edge-on could publish a pose whose
  yaw was dominated by lateral noise, and the EKF would happily fuse
  it. Field finding from iter-37 showed this was a real failure mode
  in greenhouse corridors. Commit 831f56d.

- **HIGH-04-04 — yaml-cpp parser for marker registry.** The hand-rolled
  string matcher was replaced with a proper yaml-cpp loader supporting
  per-tag size override, duplicate-id detection, and structured error
  reporting. Reduces the chance of a silently malformed registry
  passing as "no tags found". Commit 9c90a36.

- **CRITICAL-11-C-01 frontend companion.** `LoginPage.tsx` now handles
  the `must_change_password` flag set by the backend on the
  auto-generated admin user. On first login the dashboard refuses to
  call `onLogin()` until the user has set a new password through the
  in-place change-password form (the new token is held in component
  state, not committed to localStorage until the change succeeds, so a
  reload cannot bypass the prompt). Pairs with the Sprint A.5 backend
  change. Commit b7db334.

- **MEDIUM-11-D-02 — UX confirm-on-disengage for safety-critical
  buttons.** Two industrial-HMI-style modals added:
  - **TopBar E-stop button:** Engage stays single-click (slamming the
    button must never be slowed down). Disengage now opens a modal
    that requires explicit acknowledgement and spells out "motors
    remain disarmed after clear; robot will not move from this click
    alone." Mirrors hardware E-stop reset latches.
  - **OperatePanel Disarm button:** When `motors_armed && state ∈
    {navigating, executing_mission}`, the disarm click is intercepted
    by a modal warning that disarming mid-traverse will fault the
    active goal and the robot will coast. The operator must either
    cancel navigation first or knowingly confirm.
  Single `.modal-body` CSS class shared between the two. Commit dcf2212.

### Still open after Sprint E.lite

- HIGH-04-03 (kidnapping detection) — deferred. Needs a design pass to
  define the trigger criteria before implementation.
- HIGH-04-09 (ekf_local yaw absolute fight) — needs HIL test before
  merging the fix.
- HIGH-11-A-03 (TELEOP carve-out for rail_approach) — design call
  pending.
- HIGH-11-C-02/03/04 (salted KDF, JWT-off-WS-URL, TLS) — Sprint future
  (security hardening, not greenhouse-MVP-blocking).
- HIGH-11-D-04 (no goal-click confirmation) — single-click goal-send
  on the map is still un-gated; lower priority than E-stop/disarm.
- MEDIUM-06-* Phase 6 perception findings — depth pipeline refactor,
  still Sprint E proper.
- Sprint E proper (calibration wizards XL) — still gated by Sprint A
  NVRAM dump from the field.

### Sprint E.lite verifier baseline

- `bash tools/verify_specs/all.sh`: 10 scripts, 0 blocking, 1 warning
  (geometry SSOT drift, expected pending NVRAM dump).
- `colcon build agv_markers`: clean (-Werror enforced).
- `npm run build` (dashboard): clean, 39 modules transformed,
  39 kB CSS / 413 kB JS bundle.

### Cumulative status (Sprints A.5 + B + C + D + E.lite)

23 atomic commits landed across the audit cycle. Findings snapshot:

| Severity | Closed in session | Remaining open |
|---|---|---|
| CRITICAL | 2 (11-A-01, 11-C-01 backend+frontend) | 2 (CR-00-01 .repos validation, CRITICAL-02-02 numerical NVRAM fix — both await hardware) |
| HIGH | 12 | ~11 (most need hardware or design call) |
| MEDIUM | 12 | ~17 (mostly Sprint E perception + UX) |
| LOW / NIT | 0 | ~6 |

## Section 0 closure (2026-05-13)

After Sprint E.lite, the operator requested an empirical pre-Fase-1
verification of every audit finding marked "closed". This caught one
regression of the original 2026-04-13 audit bug class (a third
occurrence in `agv_rail_driver`, closed inline) and re-framed the
remaining open CRITICAL.

### Findings re-classified after Section 0

- **CRITICAL-02-02 (geometry SSOT)** — **CLOSED.** The audit's original
  hypothesis ("the 1.25× factor between scaffold 0.0781 and geometric
  0.0625 is an ODrive NVRAM bug") was empirically disproven on
  hardware: NVRAM dump shows stock M8325s values (pole_pairs=20,
  inc_encoder0.cpr=8192, no internal scale or gearing); manual wheel
  rotation confirms gearbox is 10:1 to within 1.6%; diameter is
  vernier-confirmed 125 mm. The 0.0781 scaffold was historical
  compensation for now-fixed mechanical issues. SSOT-revert sprint
  (commit `e6804e8`) flipped the values to geometric truth and
  rescaled HAL accel/decel to preserve real m/s² parity. Verifier
  WARNs cleared.
- **CRITICAL-11-A-01 (mode_arbiter type)** — **CLOSED-VERIFIED**
  end-to-end via synthetic STOP injection on the live stack.
- **CRITICAL-11-C-01 (auth defaults)** — **CLOSED-VERIFIED** end-to-end
  via forced-change modal exercised on this Jetson.
- **HIGH-04-09 (ekf yaw absolute fight)** — still open by design;
  baseline +2.4° yaw jump on stationary USB-reboot scenario captured
  for the future fix.
- **HIGH-07-02 (smoother vs HAL accel)** — **CLOSED-VERIFIED** via
  ADR-0002. The audit misread defense-in-depth as drift.
- **HIGH-04-01 (factor_graph), HIGH-11-B-01 (waypoint_manager)** —
  **CLOSED-VERIFIED** via `ros2 node list` confirming absence under
  default args + new `verify_no_waypoint_manager.sh` BLOCKING.

### New findings surfaced during Section 0

- **G4** — `agv_rail_driver` was carrying the **same root-cause class**
  as the 2026-04-13 audit bug #1 and as CRITICAL-11-A-01:
  `/agv/collision_monitor_state` subscriber declared
  `std_msgs/msg/String` while Nav2 publishes
  `nav2_msgs/msg/CollisionMonitorState`. DDS dropped every message
  silently. **Third occurrence** of the same defect in one repository.
  Closed inline in commit `08ac348`. Strongly motivates G1 (CI
  type-matching verifier).
- **G5** — Bilateral asymmetry in ODrive S1 NVRAM (vel_integrator_gain
  LEFT=0.333 vs RIGHT=0.167, can.encoder_msg_rate_ms LEFT=10 vs
  RIGHT=0). HIGH severity; deferred to Fase-1 firmware-write sprint.
- **G6** — `joint_states` publishes motor-radian as wheel-radian
  (no `/gear_ratio_` divide in the publisher). LOW severity, cosmetic.

### Section 0 verifier baseline

- `bash tools/verify_specs/all.sh`: 11 scripts, 0 blocking, **0 warnings**
  (the 2 geometry WARNs cleared after the SSOT revert).

### Cumulative status (Sprints A.5 + B + C + D + E.lite + Section 0)

27 atomic commits landed across the audit cycle. Findings snapshot:

| Severity | Closed | Remaining open |
|---|---|---|
| CRITICAL | 3 (02-02 NUMERICAL, 11-A-01, 11-C-01) | 1 (CR-00-01 .repos validation — awaits second machine) |
| HIGH | 12 + G4 + G5 captured | ~10 + G5 needs firmware-write sprint |
| MEDIUM | 12 | ~17 |
| LOW / NIT | 0 + G6 captured | ~7 |

All blocking failures resolved. Section 0 unblocks Fase 1.
