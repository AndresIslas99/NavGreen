# SUMMARY — 2026-05-13 Greenhouse Field-Readiness Audit

> One-page executive overview. Findings live in `00_inventory.md`,
> `01_foundations.md`, `02_hal.md`. Phases 3–14 are partially scoped here;
> deeper expansion happens on demand or after hardware access opens.

## Executive verdict

The 2026-04-13 governance audit successfully consolidated the workspace
into spec-as-SSOT and the verifiers are green. **The robot is not yet
field-ready** — five issues observable by static analysis alone block a
serious first-greenhouse-visit. Two of them (`CR-00-01` repo reproducibility
and `CR-00-02 / CRITICAL-02-02` geometry SSOT) would not survive a fresh
Jetson flash. Both are S-or-M effort to fix and have no engineering
unknowns. The remaining items are non-blocking but should be cleared
before MVP sign-off.

## Findings filed in this cycle

**13 findings, 3 critical / 6 high / 4 medium**, distributed across Phases
0–2 (the static-analysis-only zone). No hardware-dependent finding has
been opened because none can be verified without hardware.

| ID | Sev | Title | Phase | File:line anchor | Effort |
|---|---|---|---|---|---|
| CR-00-01 | CRITICAL | Repo not self-contained: `agv_slam` referenced but absent | 0 | `agv_full.launch.py:279` | S → L |
| CR-00-02 | CRITICAL | Wheel/track geometry has two contradictory sources of truth | 0 | `odrive_params.yaml:7-8` vs `robot_params.yaml:33` | M |
| CR-00-03 | CRITICAL | Hardcoded `/home/orza/` in launch parameter dicts | 0 | `agv_full.launch.py:546,585,701,768` | S |
| CR-00-04 | HIGH | Production network interfaces hardcoded to AGX Orin DevKit names | 0 | `agv_start.sh:53,90` | S |
| CR-00-05 | HIGH | License divergence (`agv_rail_approach` Proprietary, others MIT) undeclared | 0 | `agv_rail_approach/package.xml` | S |
| CR-00-06 | HIGH | `agv_waypoint_manager` bypasses localization gate (restated from spec) | 0 | `waypoint_manager_node.cpp:109` | M |
| HIGH-01-01 | HIGH | `base_footprint` as child of `base_link`, contrary to community pattern | 1 | `agv_base.xacro:33-39` | S |
| HIGH-01-02 | HIGH | Camera frame transform lives in external `agv_slam` package | 1 | `agv_full.urdf.xacro:14-15` | M |
| MEDIUM-01-03 | MEDIUM | `static_transform_publisher` masks vibration-induced extrinsic drift | 1 | depends on HIGH-01-02 | M |
| MEDIUM-01-04 | MEDIUM | No master-clock discipline at boot (chrony not invoked) | 1 | `agv_start.sh` | S |
| MEDIUM-01-05 | MEDIUM | HIL mixes wall-clock and sim-clock topics with implicit assumption | 1 | `agv_full.launch.py:86-95` | S |
| LOW-01-06 | LOW | TF single-owner invariant not enforced by verifier | 1 | proposal | S |
| HIGH-02-01 | HIGH | ros2_control cutover plan needed (post-MVP), keep `agv_hw_interface` in CI | 2 | `src/agv_hw_interface/` | tracking |
| CRITICAL-02-02 | CRITICAL | Geometry SSOT — runtime YAML overrides C++ defaults silently | 2 | `odrive_can_node.cpp:13-14,31` vs `odrive_params.yaml` | M |
| HIGH-02-03 | HIGH | URDF chassis inertia is literal `1.0` placeholder | 2 | `agv_base.xacro:25-29` | S |
| HIGH-02-04 | HIGH | ZED depth+IMU pipeline config invisible from this repo | 2 | `agv_slam` is external | M → L |
| MEDIUM-02-05 | MEDIUM | CAN bitrate docs vs script comment disagree (250 vs 500 kbps) | 2 | `project.yaml:69` vs `agv_start.sh:241` | S |
| MEDIUM-02-06 | MEDIUM | `nav2_params.yaml` header still says "RegulatedPurePursuit", code is MPPI | 2 | `nav2_params.yaml:11-12` | S |
| HIGH-02-07 | HIGH | Possible double-counted `gear_ratio` (ROS × firmware) | 2 | `odrive_params.yaml:9` + `odrive_can_node.cpp:28-31` | S |

CR-00-02 and CRITICAL-02-02 are **the same root cause** in two phases (Phase 0
spots the divergence; Phase 2 traces the runtime-resolution mechanism and
adds the migration plan).

## Priority queue (recommended order of remediation)

### Sprint A — repro and geometry (1 week)
The minimum to safely run a hardware test of the rest.

1. **CR-00-01** — Author `agv_greenhouse.repos`, document `vcs import` in README. Decide vendoring strategy for `agv_slam`. **Without this, none of the other findings can be reproduced on a fresh system.**
2. **CRITICAL-02-02 (= CR-00-02)** — Physical re-measurement of wheel radius and track width. Centralize on `robot_params.yaml`. Refactor `odrive_can_node` to refuse parameter defaults. Add `verify_geometry_ssot.py`.
3. **CR-00-03** — Replace hardcoded `/home/orza/` with `os.environ.get('AGV_DATA_DIR', …)` in the four launch dicts.
4. **HIGH-02-07** — Read ODrive NVRAM gear_ratio and add boot-time invariant check.

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
