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

**42 findings total, 2 critical / 16 high / 20 medium / 4 low** across
Phases 0–10 (1 NIT marked separately for MEDIUM-07-06). The HAZOP
skeleton is shipped as `docs/safety/HAZOP_skeleton.md` (not numbered as
a finding — it's a deliverable). Phases 3 (calibration), 5 (mapping),
8 (greenhouse behaviors), 11–14 (GUI, sim, CI, docs), and 15 (field)
remain to be filed — see §F.

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
