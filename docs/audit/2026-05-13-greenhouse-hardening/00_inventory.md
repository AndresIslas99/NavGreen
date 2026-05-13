# Phase 0 — Inventory & Cartography

> Phase 0 goal (per prompt): "Reporte produce visualización clara del
> workspace, un humano nuevo puede entender el sistema en < 30 min
> leyéndolo."

This file is the **map**: what is in the workspace, what depends on what,
where the contracts live, and which assumptions a fresh reader is licensed
to make. Critical findings discovered while building the map are filed
here so they are not lost — deeper analysis goes in the phase-specific
files that follow.

**Date**: 2026-05-13 — **Branch**: `claude/amr-security-audit-gPtCd` — **Working tree**: clean
**Prior audit context**: [`../2026-04-13-full-audit.md`](../2026-04-13-full-audit.md) (33 KB, governance audit; 6 latent bugs identified, 5 marked FIXED in specs)
**Spec verifier baseline**: `bash tools/verify_specs/all.sh` → 9 scripts run, 0 blocking failures, 1 warning (vendored `zed-ros2-wrapper` absent from this clone, expected).

---

## A. Workspace cartography

### A.1 Top level

```
agv-greenhouse/
├── AGENTS.md, AGENT_INSTRUCTIONS.md, CLAUDE.md       — agent governance
├── README.md, STATUS.yaml                            — project descriptor
├── agents/registry.yaml                              — agent roles
├── policies/engineering_rules.md                     — Rules 0–9
├── specs/                                            — SSOT (8 YAMLs, 41 KB interfaces.yaml)
│   ├── README.md
│   ├── project.yaml, state_machine.yaml,
│   │   launch_sequence.yaml, persistence.yaml,
│   │   interfaces.yaml, acceptance.yaml, hmi_api.yaml
├── docs/                                             — human + audit docs
│   ├── architecture.md, hardware_setup.md, …
│   └── audit/2026-04-13-*.md, audit/2026-05-13-greenhouse-hardening/  (this folder)
├── src/                                              — 24 ROS 2 packages
├── web/agv_dashboard/                                — React 19 + Vite + Leaflet operator HMI
├── fleet/                                            — VDA 5050 + MQTT (out of MVP scope)
├── tools/                                            — verify_specs/, calib_runs/
└── scripts/                                          — kill_hil_stack.sh (dev)
```

### A.2 Packages in `src/` (24 total)

| Pkg | Lang | Test files | Werror | License | Notes |
|---|---|---|---|---|---|
| agv_behaviors | C++17 | 1 | ✅ | MIT | BehaviorTree.CPP v3 |
| agv_bringup | Python launch | 0 | n/a | MIT | 3 production entry points |
| agv_description | xacro | 1 | n/a | MIT | URDF entry: `agv_full.urdf.xacro` |
| agv_factor_graph | C++17 + Py dev | 0 | ✅ | MIT | GTSAM parallel (publish_tf=false) |
| agv_hil_bridges | Python | 0 | n/a | MIT | HIL only — `dev_only: true` in TASK |
| agv_hw_interface | C++17 + Py dev | 2 | ✅ | MIT | ros2_control SystemInterface (alt path to ODrive; OPT-IN, not production default) |
| agv_image_server | C++17 | 1 | ✅ | MIT | MJPEG :8091 |
| agv_integration_tests | Python | 24 | n/a | MIT | HIL test harness |
| agv_interfaces | IDL | 0 | n/a | MIT | 2 msg + 6 srv |
| agv_localization_init | C++17 + Py dev | 0 | ✅ | MIT | Auto-init orchestrator cascade |
| agv_map_manager | C++17 + Py dev | 1 | ✅ | MIT | Save/Load + zones.json + .area copy |
| agv_markers | C++17 | 1 | ✅ | MIT | AprilTag pose factor |
| agv_mode_arbiter | C++17 + Py dev | 2 | ✅ | MIT | 3-source cmd_vel multiplexer FSM |
| agv_navigation | YAML/Py launch | 0 | n/a | MIT | Nav2 SmacPlanner2D + MPPI |
| agv_odrive | C++17 + Py dev | 5 | ✅ | MIT | CAN driver + wheel_odom 50 Hz |
| **agv_rail_approach** | C++17 | 1 | ✅ | **Proprietary (Orza)** | **License divergence — see CR-00-05** |
| agv_rail_detector | C++17 + Py dev | 1 | ✅ | MIT | Depth → BEV → RANSAC rail tube |
| agv_rail_driver | C++17 + Py dev | 1 | ✅ | MIT | Longitudinal-only, wz=0 hard |
| agv_safety | C++17 + Py dev | 3 | ✅ | MIT | `safety_supervisor` + `cmd_vel_gate` |
| agv_scan_mapper | C++17 | 1 | ✅ | MIT | Live `/agv/live_map` overlay |
| agv_sensor_fusion | C++17 + Py dev | 3 | ✅ | MIT | dual-EKF + IMU filter + slip detector + caster dwell |
| agv_ui_backend | TypeScript | 0 | n/a | MIT | Express + rclnodejs + WS |
| agv_waypoint_manager | C++17 | 1 | ✅ | MIT | Mission CRUD + dispatch (bypasses backend gates — known gap) |
| agv_zone_detector | C++17 + Py dev | 1 | ✅ | MIT | Aisle/corridor classifier |

**Verification**: `grep -rn 'Werror' src/*/CMakeLists.txt` lists 17 hits across the 17 C++ packages — `-Werror` is enforced. The previous Explore agent's claim of "0 Werror enforcement" is wrong; `verify_werror.sh` passes precisely because the flag exists in every C++ package's `CMakeLists.txt:5`.

### A.3 Launch entry points

Only **three** production launch files in `agv_bringup/launch/`:

| File | Selected when | TF owner for `map→odom` | Safety chain |
|---|---|---|---|
| `agv_full.launch.py` | `AGV_MODE=real` (default) | `ekf_global` | active iff `has_map` (collision_monitor + cmd_vel_gate) |
| `agv_full.launch.py hil_mode:=true` | `AGV_MODE=hil_full` | `ekf_global` | active iff `has_map` |
| `agv_mapping.launch.py` | `AGV_MODE=mapping` | cuVSLAM (different invariant!) | none |
| `agv_hil_full.launch.py` | `AGV_MODE=hil` | `ekf_global` (sim provides `/visual_slam/tracking/odometry`) | none |

10+ previously-existing launch files were deleted in the 2026-04-13 audit (Fase 6 bug #4); cross-check confirms no dead launch files remain.

### A.4 External (non-vendored) dependencies the workspace expects to find

These packages are referenced by the production launch but are **not in `src/`**. They must be installed elsewhere (`/home/orza/ros2_ws/install/` per `agv_start.sh:165`).

| Package | Role | Risk if missing |
|---|---|---|
| `agv_slam` | cuVSLAM + nvblox + ZED wrapper integration | Production launch fails at `IncludeLaunchDescription(... agv_slam.launch.py ...)` — fatal |
| `isaac_ros_visual_slam`, `isaac_ros_apriltag` | Isaac ROS pipeline | cuVSLAM, AprilTag detection fail |
| `zed-ros2-wrapper`, `zed_components`, `zed_msgs` | ZED 2i driver (PATCHED — re-reads area_memory_db_path live) | Cannot get camera, IMU, depth |
| `slam_toolbox` | Loop closure (optional) | Drift grows unbounded over long runs |
| `nav2_*` (collision_monitor, smoother, bt_navigator, …) | Navigation stack | All goals fail |
| `pointcloud_to_laserscan` | Depth → 2-D scan | Costmap loses obstacle source; collision_monitor degraded |
| `foxglove_bridge` | Diagnostic WS on :8765 (opt-in) | Engineering visibility lost |
| `apriltag_ros` (community / Christian Rauch fork) | Tag detection | Marker correction silent |
| `robot_localization` | EKF nodes | No fusion possible |
| `robot_state_publisher`, `xacro` | URDF → TF | TF tree collapses |

**See finding CR-00-01.** A fresh clone cannot boot or build without procuring these. The repo has no `*.repos` file, no submodules, no vendoring policy, no `dockerfile`. Reproducibility is implicit.

### A.5 Front-end + fleet

- `web/agv_dashboard/` — React 19 + Vite + Leaflet + TypeScript 5.9 (operator HMI per `specs/hmi_api.yaml`).
- `fleet/agv_fleet_manager`, `fleet/agv_vda5050_adapter`, `fleet/mosquitto` — Node.js + Express + MQTT 5. **Out of MVP scope** per `specs/project.yaml#scope.out_of_scope` ("VDA 5050 interoperability", "fleet management").

---

## B. Modes, runtime layers, launch DAG

### B.1 The four-layer mode matrix (verbatim from spec)

[`specs/state_machine.yaml`](../../../specs/state_machine.yaml) is authoritative. Summary:

| Layer | Variable | Authority | Live-switchable | Source |
|---|---|---|---|---|
| 1. systemd | `AGV_MODE` ∈ {real, mapping, hil} | operator (`systemctl edit`) | no — different launch file | `agv.service` env |
| 2. launch | `has_map` ∈ {with_map, without_map} | derived at boot from `AGV_MAP` resolution | no — different topology | `agv_full.launch.py:74` |
| 3. runtime | `currentMode` ∈ {teleop, mapping, nav} | operator (dashboard) | **yes** (WS message) | `agv_ui_backend::setMode` |
| 4. derived | `RobotState` ∈ {idle, ready, mapping, navigating, executing_mission, blocked, e_stop, fault} | sensors | observation only | `state_machine.ts::deriveState` |

The spec's six `valid_combinations` enumerate the supported tuples; "any other combination either never happens or silently produces broken behavior." That asymmetry — invalid combinations producing silence, not crashes — is the root cause of the historical teleop-broken incident (2026-04-13 audit bug #1) and remains a class of risk.

### B.2 Launch DAG (compressed)

```
t = 0.0 s  robot_state_publisher      (TF base_link → wheels)
           odrive_can_node            (cmd_vel → motors, /agv/wheel_odom 50 Hz)
           pointcloud_to_laserscan    (/agv/zed/point_cloud → /agv/scan)
           image_server               (:8091 MJPEG)
           scan_grid_mapper           (/agv/live_map)
t = 3.0 s  agv_slam (external)        (cuVSLAM, TF DISABLED via /**: key in cuvslam_greenhouse.yaml)
t = 3.5 s  imu_filter                 (Butterworth 10 Hz / 5 Hz cutoffs)
t = 3.8 s  wheel_slip_detector        (inflates wheel_odom covariance on caster slip)
           caster_dwell_advisor       (/agv/caster/dwell_state — no consumer yet)
t = 4.0 s  ekf_local                  (odom → base_link, 50 Hz, SOLE OWNER)
           ekf_global                 (map → odom, 10 Hz, SOLE OWNER)
           fusion_monitor             (/agv/pose, /diagnostics)
t = 4.5 s  factor_graph (parallel)    (publish_tf=false, validation only)
t = 5.0 s  slam_toolbox_localization  (transform_publish_period=0 → TF DISABLED)
           map_manager                (save/load + zones.json + .area copy)
           waypoint_manager           (mission CRUD — bypasses backend gates)
t = 6.0 s  Nav2 stack  [has_map]      (SmacPlanner2D + MPPI + collision_monitor + velocity_smoother)
t = 6.5 s  agv_safety stack [has_map] (safety_supervisor + cmd_vel_gate)
t = 7.0 s  apriltag + marker_correction + rail_approach [enable_markers]
           zone_detector + rail_detector + mode_arbiter + rail_driver [has_map]
           auto_init_orchestrator    [has_map]
           behavior_executor          [enable_behaviors — default false, NEVER ENABLED]
t = 7.5 s  foxglove_bridge            [enable_foxglove_bridge — default false]
t = 8.0 s  teleop_backend (Node.js)   (REST :8090, WS /ws/control)
```

The spec ([`specs/launch_sequence.yaml`](../../../specs/launch_sequence.yaml)) is authoritative; this is a compressed view for orientation.

### B.3 cmd_vel chain (verbatim — verified)

```
Dashboard joystick / WS → teleop_server.cmd_vel
              ↓ Phase-2 (arbiter): teleop_server publishes only if mode_arbiter source == NONE
                       ↓ Phase-2 sources: cmd_vel_nav | cmd_vel_approach | cmd_vel_rail
              ↓ mode_arbiter relays
       /agv/cmd_vel
        ↓ has_map=false ───────────────────────────→ agv_odrive_node
        ↓ has_map=true
       velocity_smoother  (Nav2)
        ↓ /agv/cmd_vel_smoothed
       collision_monitor (Nav2)         polygons: slowdown + stop, scan + pointcloud
        ↓ /agv/cmd_vel_collision_safe   (renamed 2026-04 from cmd_vel_safe)
       cmd_vel_gate (agv_safety)        watchdog 0.5 s; e_stop + hardware_estop force zero
        ↓ /agv/cmd_vel_safe
       agv_odrive_node → CAN → wheels
```

**Single owner of `map→odom`**: `ekf_global`.
**Single owner of `odom→base_link`**: `ekf_local`.
The cuVSLAM TF disable uses the `/**:` YAML key (NOT the node name) — a hidden convention. Misuse silently leaves dual publishers.

---

## C. TF tree (claimed vs verifiable from code)

```
                map
                 │  ekf_global @ 10 Hz       (single publisher)
                odom
                 │  ekf_local  @ 50 Hz       (single publisher)
             base_link
            ╱   │   ╲
   left_wheel  │  right_wheel               (URDF; robot_state_publisher)
               │
        base_footprint (z = -0.200 m)        (URDF; child of base_link)
               │
        [ZED frames] ← NOT IN URDF           (published by agv_slam, which is
                                              external and not in this repo)
```

**Issues flagged for Phase 1 deep dive**:
1. `base_footprint` is a **child** of `base_link` at z = -0.200 m. REP-103 has no opinion on the ordering, but REP-105 specifically lists `… → base_link → base_footprint` is **non-standard** for AGVs: most platforms (Husky, Jackal, Turtlebot4) put `base_footprint` at the **root** of the chain because it is the kinematic origin on the floor. The current order works for Nav2 (which uses `robot_base_frame: base_link`) but breaks the convention multi-robot fleets and external tooling rely on. Tracked as `HIGH-01-XX` in Phase 1.
2. ZED frames are not in URDF. Camera-to-base extrinsics live in a `static_transform_publisher` inside `agv_slam` (external). Any in-field disturbance to the camera mount must be calibrated and applied **outside this repo**, which violates the workspace's own SSOT principle (`specs/persistence.yaml` lists configs but does not cover external static TFs).
3. The `/**: ` YAML key invariant for cuVSLAM TF disable is a single-line trap: using the node name silently re-enables a competing publisher. The spec annotates this; it is not enforced by any verifier.

---

## D. Persistence summary

[`specs/persistence.yaml`](../../../specs/persistence.yaml) lists 14 artifacts. Key facts for a fresh reader:

- **Canonical base**: `${AGV_DATA_DIR}` (default `/home/orza/agv_data`).
- **Per-map quadruplet**: `<X>.yaml + <X>.pgm + <X>_cuvslam/ + <X>.area + <X>_meta.json`. Save Map must produce all five atomically; Load Map must consume all five.
- **Boot marker**: `~/.agv/last_map` (plain text, no extension). Read by `agv_start.sh` *before* ROS is sourced.
- **Whitelisted hardcodes**: 4 locations still hardcode `/home/orza/agv_data/maps/.current.area` — see `verify_no_hardcoded_paths.sh` whitelist. Cleanup tracked in the prior audit, not in this one.

---

## E. Critical findings emerging from Phase 0

Five findings rise to the surface from the cartography work alone. Each is filed once here; deeper analysis lives in the per-phase files that follow.

---

### CR-00-01 — Repository is not self-contained: `agv_slam` is required but absent
**File(s)**: `src/agv_bringup/launch/agv_full.launch.py:279,128`; `src/agv_bringup/launch/agv_mapping.launch.py:116`; `src/agv_bringup/launch/agv_hil_full.launch.py:298`; `src/agv_bringup/scripts/agv_start.sh:165` (`source /home/orza/ros2_ws/install/setup.bash`).
**Category**: architecture / reproducibility.
**Symptom**: All three production launch files call `FindPackageShare('agv_slam')` to include `agv_slam.launch.py`. `find /home/user/agv-greenhouse -type d -name agv_slam` returns nothing. The expected sibling workspace at `/home/orza/ros2_ws/install/` is hard-wired into the boot script.
**Analysis**: `agv_slam` packages cuVSLAM + nvblox + ZED wrapper integration + a `static_transform_publisher` for the camera frame (per the URDF header comment at `agv_full.urdf.xacro:14-15`). The repo is layered: this workspace assumes a **sibling install** that contains `agv_slam` and the vendored Isaac ROS packages. There is no `*.repos` file (no `vcs import` reproducibility), no submodule, no Docker context, no documentation of which sibling repo provides `agv_slam`. A new engineer cloning this repo cannot build it; a CI runner cannot validate it; a backup of the robot Jetson can be restored only if the sibling workspace was also backed up.
**Greenhouse impact**: At first field visit, if the SD card / NVMe is reflashed, the team will discover this at the worst possible moment. There is no documented recovery procedure. Production stack will fail to launch and the error message (`agv_slam not found`) gives no clue that a separate workspace is needed.
**Benchmark**: ROS-Industrial publishes a `*.repos` for every distribution. Autoware Universe uses `vcs import` from `autoware.repos`. Open Navigation publishes a docker base image with all upstream deps pinned. ANYbotics / ETH RSL maintain a per-distribution `dependencies.repos`. The minimum bar is a `agv_greenhouse.repos` listing the URLs and pinned SHAs of every external dependency.
**Recommendation** (3 options, by ascending cost):
1. **Quick win (S)**: Add `agv_greenhouse.repos` to the repo root listing `agv_slam`, `zed-ros2-wrapper`, all `isaac_ros_*`, `slam_toolbox`, `nav2_*`, `apriltag_ros`, `foxglove_bridge`, `robot_localization`, `pointcloud_to_laserscan` with pinned SHAs. Document `vcs import src < agv_greenhouse.repos` in README.
2. **Medium (M)**: Vendor `agv_slam` as a submodule (or absorb into `src/` if licensing allows). The cuVSLAM/Nvblox config is small; the only non-trivial dependency is the patched `zed-ros2-wrapper`.
3. **Long (L)**: Reproducible build via a Dockerfile + `colcon mixin` setup; CI matrix on `industrial_ci` validates that the repo + `.repos` file builds from a clean Jetson L4T image.
**Acceptance criterion**: `git clone <repo> && vcs import src < agv_greenhouse.repos && colcon build` produces a runnable workspace on a fresh Ubuntu 22.04 + JetPack 6 system with no manual steps. CI runs this end-to-end on every PR.
**Effort**: S (option 1) → L (option 3).
**Prerequisites**: none.

---

### CR-00-02 — Wheel & track geometry has two contradictory sources of truth
**File(s)**:
- `src/agv_description/urdf/agv_full.urdf.xacro:20` — `wheel_radius default="0.0625"` (62.5 mm)
- `src/agv_description/config/robot_params.yaml:33` — `radius: 0.0625` + `track_width: 0.735`
- `src/agv_odrive/config/odrive_params.yaml:7` — `wheel_radius: 0.0781` (calibrated 2026-04-08) + `track_width: 0.960` (calibrated 2026-04-08)
- `src/agv_navigation/config/nav2_params.yaml:250,322` — footprint `[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]` (width 0.74 m)

**Category**: calibration / bug.
**Symptom**: Three configuration files declare wheel radius and track width with **incompatible values**. The URDF (62.5 mm) and `robot_params.yaml` (called out as "Single Source of Truth" in its own header) agree on 62.5 mm radius and 0.735 m track width. The ODrive YAML — annotated `calibrated 2026-04-08 (ratio 2.66/2.13 from 3-trial test)` for the radius and `calibrated 2026-04-08 (effective, from 2 spin trials)` for the track — uses 78.1 mm radius and 0.960 m effective track width. The Nav2 footprint was sized to the URDF, not to ODrive.
**Analysis**: The ODrive value is the post-UMBmark "effective" calibration (Borenstein-style), which by construction can differ from the geometric measurement because it absorbs systematic error (encoder ticks/rev mismatch, gear backlash, tire deformation). Either:
- The URDF is **the geometric ground truth** and ODrive **introduces a calibration bias** that should NOT propagate to URDF/Nav2 (the standard practice — UMBmark calibration is private to the odometry layer). In this case `robot_params.yaml`'s SSOT claim is intact, but the ODrive YAML's variable names (`wheel_radius`, `track_width`) are a misnomer — they should be named `effective_radius` / `effective_track_width` to make clear they are odometry-tuning parameters, not physical measurements.
- OR the URDF was simply never updated after the 2026-04-08 wheel/measurement change and is stale. In this case the URDF is incorrect by ~25 % radius and ~23 % width, and Nav2's footprint is correspondingly wrong.
The 25 % radius difference is too large to be UMBmark-only. UMBmark typically yields corrections in the 1–5 % range; 25 % implies a different wheel was installed or the original CAD value was wrong. The 0.735→0.960 m track delta (+23 %) is similar in magnitude. **The most likely explanation is that the physical platform was modified (new wheels / new axles) and only `odrive_params.yaml` was updated.**
**Greenhouse impact**:
- **Collision risk**: Nav2's footprint is 0.74 m wide. If real track is 0.960 m, wheels stick out 0.11 m on each side **outside the costmap footprint**. Nav2 plans through corridors that physically scrape the cuna de plantas. The greenhouse uses 1.0–1.5 m corridors; an undeclared 0.22 m of wheel does not survive.
- **Odometry drift**: with a 25 % wheel radius error, every 1 m of commanded distance becomes 0.80 m of actual travel (or vice versa). The dual-EKF will fight this against the IMU yaw rate and produce a degraded pose estimate that wanders.
- **MPPI controller**: the controller models the robot kinematics assuming Nav2's footprint and robot params. Mismatched values degrade tracking and produce undamped oscillation.
- **HIL ↔ real divergence**: `agv_hil_bridges` is documented (`src/agv_hil_bridges/CLAUDE.md:22-24`) to use the *same* `wheel_radius` / `track_width` as `odrive_params.yaml`. So HIL is consistent with calibrated ODrive but inconsistent with URDF, which means HIL is **not testing what Nav2 sees**.
**Benchmark**: Nav2 community recommends a single physical-parameter YAML loaded into both `robot_state_publisher` (via `xacro --inorder`) and the controller config — see Steve Macenski's nav2_bringup pattern. ros2_control's `diff_drive_controller` reads `wheel_separation` and `wheel_radius` from the controller YAML, and the URDF macro references the same numbers via xacro args. Both ETH RSL's ANYmal and Clearpath Husky-Jackal stacks centralize wheelbase/track in a single YAML and propagate via xacro.
**Recommendation**:
1. **Decide which value is the physical ground truth.** Physically re-measure with a caliper + tape measure: distance between wheel-tread centers (track), and wheel rolling-circumference / 2π (radius). Document the measurement procedure in `docs/calibration/physical_geometry.md` with photos.
2. **Update `robot_params.yaml`** to that physical truth. Update the URDF defaults to read from it via xacro args (currently `agv_full.urdf.xacro:20-25` hardcodes defaults; replace with `<xacro:arg name="..."/>` reading from a single `robot_params.yaml` loaded by `description.launch.py`).
3. **Rename ODrive parameters** to `effective_radius` / `effective_track_width` if they are odometry-calibration values. Document the UMBmark output as the *difference* from physical, not the absolute physical value. If the 25 % delta is real (not UMBmark — actually new wheels), then both parameter sets must agree at the physical value, with no per-package overrides.
4. **Update Nav2 footprint** to match physical track + 2× wheel-tread half-width + safety margin. Currently footprint front edge x=+0.50 vs chassis front x=+0.70 in URDF — also off by 0.20 m. Footprint should be `physical_chassis + margin` not `physical_chassis − 0.20 m`.
**Acceptance criterion**: `robot_params.yaml` is the only file declaring wheel/track values. URDF, Nav2 footprint, ODrive odometry kinematics, and HIL bridge all derive from it (verified by `tools/verify_specs/verify_geometry_ssot.py` — to be written). UMBmark calibration values, if any, live in a separate `wheel_odom_calibration.yaml` with explicit "effective" naming. The 100 m closed-loop drift on hardware is < 0.5 % with AprilTags and < 2 % without (per Phase 4 acceptance).
**Effort**: M (one day of measurement + half-day of refactor + verification).
**Prerequisites**: none, but blocks every navigation accuracy claim.

---

### CR-00-03 — Hardcoded `/home/orza/...` paths inside launch parameter dicts
**File(s)**:
- `src/agv_bringup/launch/agv_full.launch.py:546` — `'runtime_registry_file': '/home/orza/agv_data/runtime_markers_registry.yaml'`
- `src/agv_bringup/launch/agv_full.launch.py:585` — same path, second occurrence
- `src/agv_bringup/launch/agv_full.launch.py:701` — `{'map_dir': '/home/orza/agv_data/maps'}`
- `src/agv_bringup/launch/agv_full.launch.py:768` — `additional_env={'AGV_DATA_DIR': '/home/orza/agv_data', ...}`
- `src/agv_bringup/scripts/agv_start.sh:165,175,179,187,189` — multiple hardcodes including `MAP_DIR="${AGV_MAP_DIR:-/home/orza/ros2_ws/install/agv_navigation/share/agv_navigation/maps}"`

**Category**: portability / reproducibility.
**Symptom**: The verifier `verify_no_hardcoded_paths.sh` passes because it whitelists known paths and only greps source files. Paths embedded in **Python dictionaries inside launch files** are not currently flagged. Production deployment is hard-pinned to a `/home/orza/` home directory.
**Analysis**: The Jetson AGX Orin DevKit (current dev target) was set up with username `orza`. The production target is Jetson Orin NX 16 GB, which may or may not have the same username. The code mostly uses `AGV_DATA_DIR` with a fallback (`agv_full.launch.py:68` does `os.environ.get('AGV_DATA_DIR', '/home/orza/agv_data')`), but the four launch-parameter dicts above and the boot script hardcode the literal — there's no env-var fallback.
**Greenhouse impact**: Reflashing or migrating the robot to a clean Jetson with a different username silently breaks marker registry loading and last-known-pose recovery (which depend on `runtime_markers_registry.yaml` and `meta.json` paths). Field technicians will not understand why AprilTag-defined waypoints are missing or why the robot cannot resume after reboot.
**Benchmark**: Apollo Cyber RT and Autoware Universe enforce zero hardcoded user paths at lint time. Nav2 reference launches use `LaunchConfiguration` substitutions and `PathJoinSubstitution([FindPackageShare(...), ...])` exclusively.
**Recommendation**:
1. **Replace all 4 launch-param hardcodes** with `LaunchConfiguration('data_dir')` derived from `os.environ.get('AGV_DATA_DIR', '/home/orza/agv_data')` once at the top of `generate_launch_description()` and reused.
2. **Refactor `agv_start.sh`** to compute `AGV_DATA_DIR`, `AGV_MAP_DIR`, `ROS_WS` from `${HOME}` if not set, and document the env override in the systemd unit. Move the `source /home/orza/ros2_ws/install/setup.bash` to use `${ROS_WS:-${HOME}/ros2_ws}/install/setup.bash`.
3. **Extend `verify_no_hardcoded_paths.sh`** to also grep Python literal strings inside launch files (`*.launch.py`) for `/home/`, with explicit allow-list for the fallback `default=` cases.
**Acceptance criterion**: `grep -rn '/home/orza' src/` returns 0 hits (or only in comments / docs explicitly noted). Re-flashing to a Jetson with username `agv` requires zero source modifications.
**Effort**: S.
**Prerequisites**: none.

---

### CR-00-04 — Production network interfaces are hardcoded to AGX Orin DevKit names
**File(s)**:
- `src/agv_bringup/scripts/agv_start.sh:53,90` — `for iface in eno1 wlP1p1s0; …; CYCLONE_CANDIDATES="eno1 wlP1p1s0"`.

**Category**: portability / bug.
**Symptom**: `agv_start.sh` generates `/tmp/agv_cyclonedds_runtime.xml` at boot by enumerating exactly two interface names: `eno1` and `wlP1p1s0`. Both are AGX Orin DevKit specific (the WiFi card is `wlP1p1s0` on the AGX, but the Orin NX 16 GB module on a third-party carrier almost certainly exposes a different name — `wlan0`, `wlx*`, etc.).
**Analysis**: This file is a careful piece of work — it specifically handles rfkill state, the dual operstate/carrier check, and dynamic XML generation to dodge CycloneDDS's hard requirement that listed interfaces exist. The carefulness is undermined by hardcoding the candidate list. There's no `${AGV_NETWORK_IFACES:-eno1 wlP1p1s0}` env override.
**Greenhouse impact**: Moving from the AGX Orin 64 GB development target to the Orin NX 16 GB production target (declared in `specs/project.yaml:deployment_targets.production`) will silently fall back to the localhost-only DDS config (`agv_start.sh:121-124`). Operator dashboard on tablet **cannot discover the robot** because Cyclone is listening only on `localhost`. The fallback is correct and safe (no SIGABRT), but it is silent — the operator just sees "robot offline" with no diagnostic clue.
**Benchmark**: Most production robots (Husky, Spot, Locus) enumerate interfaces dynamically — Spot does this via `bosdyn-system-helpers`. Robotec.ai's launch scripts pull from `/etc/network/interfaces` or `nmcli`. Apollo Cyber RT lists interfaces by capability (`HAS_IPV4`), not by name.
**Recommendation**:
1. **Read interface candidates from env / config**, with the current AGX-specific list as a fallback. `CYCLONE_CANDIDATES="${AGV_NETWORK_IFACES:-$(ls /sys/class/net | grep -vE '^(lo|docker|veth|can|usb)' | tr '\n' ' ')}"` (auto-enumerate, exclude virtual + non-IP).
2. **Add an end-of-boot diagnostic**: if no whitelisted iface is up, emit a clear `journalctl` line and a dashboard event so the operator sees "DDS listening on localhost only — no LAN interface".
3. **Document the override** in `docs/deployment/network.md` (new file) — what env var to set, how to test.
**Acceptance criterion**: Booting on a Jetson Orin NX with a `wlan0` interface in `AGV_NETWORK_IFACES` env produces `agv_cyclonedds_runtime.xml` listing that interface, and the dashboard can reach the robot on the LAN without manual editing.
**Effort**: S.
**Prerequisites**: none.

---

### CR-00-05 — License divergence within the workspace, undeclared
**File(s)**:
- `src/agv_rail_approach/package.xml` — `<license>Proprietary</license>`, `<maintainer email="dev@orza.mx">Orza</maintainer>`.
- All other 23 packages — `<license>MIT</license>`, `<maintainer>` set to project lead.

**Category**: governance / docs.
**Symptom**: One package (`agv_rail_approach`) is Proprietary; everything else is MIT. There is no workspace-level `LICENSE.md` and no `NOTICE` file enumerating which components are under which license. `README.md` does not mention licensing at all.
**Analysis**: `agv_rail_approach` is greenhouse-customer-specific IP (it's the AprilTag-guided dock controller for rail entry). Mixing MIT and Proprietary is legitimate — many real robotics products do — but it must be **declared**. The current ambiguity creates downstream risks:
- Someone forks the repo assuming all of it is MIT and ships a derivative work that includes the proprietary node.
- A new package added to `src/agv_*` defaults to MIT (the workspace template implies it), but if the new code wraps or extends `agv_rail_approach`, it inherits the proprietary obligations.
- CI / `verify_specs/verify_canonical_sources.sh` does not check license consistency.
**Greenhouse impact**: Indirect, but real — when the customer's legal team reviews the deployment, the mixed licensing without a manifest will block sign-off until clarified.
**Benchmark**: Autoware publishes `LICENSE.md` listing every package and its license. Open Robotics publishes a `NOTICE` file per repo. Reuse / SPDX expects an `SPDX-License-Identifier:` comment header on every source file.
**Recommendation**:
1. **Add `LICENSE.md`** at the repo root: project-level MIT for the workspace + an explicit carve-out for `agv_rail_approach` (Proprietary, owned by Orza, contact `dev@orza.mx`).
2. **Add SPDX headers** to every source file:
   - `// SPDX-License-Identifier: MIT` for MIT files
   - `// SPDX-License-Identifier: LicenseRef-Orza-Proprietary` for the proprietary package
3. **Extend `verify_canonical_sources.sh`** (or add `verify_license.sh`) to assert that every source file under `src/agv_*/{src,include}` has a SPDX header that matches its `package.xml` declaration.
**Acceptance criterion**: A new external contributor reading `LICENSE.md` knows exactly which parts they can fork and which they cannot. `verify_license.sh` blocks PRs that add a source file without an SPDX header.
**Effort**: S (LICENSE.md + headers) → M (verifier + retroactive header pass).
**Prerequisites**: none.

---

### CR-00-06 — `agv_waypoint_manager` bypasses the localization gate (known gap, restated)
**File(s)**:
- `src/agv_waypoint_manager/src/waypoint_manager_node.cpp:109` (cited in `specs/interfaces.yaml#actions`).
- `specs/state_machine.yaml#invariants.nav_goal_requires_localization` — `known_gap` block.

**Category**: bug / safety.
**Symptom**: The single-goal path through `agv_ui_backend::sendNavGoal` checks the localization state (`localization.action != 'FAILED'`) before dispatching to Nav2. The mission-execution path goes directly from `waypoint_manager_node` to the Nav2 action client, bypassing that gate. The spec acknowledges this as a known gap tracked for "Fase 8".
**Analysis**: The spec is honest about this and the 2026-04-13 audit already documented it. It is restated here because it has not closed and it is a direct safety/operational risk in greenhouse missions.
**Greenhouse impact**: An operator clicks "Run Mission" on a 12-waypoint route through the rows. If localization has silently degraded to FAILED (e.g., the AprilTag at the start was occluded), Nav2 receives map-frame goals with a stale `map→odom` offset and the robot physically navigates to phantom coordinates. The 2026-04-13 trip log records a 2.98 m goal that advanced 0.67 m in 3 m 30 s before manual cancellation — for a single goal. A 12-waypoint mission compounds the risk.
**Recommendation**:
Two options:
1. **Refactor (preferred)**: Mission executor sends an HTTP POST to `agv_ui_backend/api/nav/goal_internal` instead of using its own action client. Reuses the existing gate. Documented in `specs/hmi_api.yaml`.
2. **Local gate (quick fix)**: Have `waypoint_manager_node` subscribe to `/agv/localization/state` and reject `execute_mission` if state is `FAILED`. Lighter, but duplicates the logic that lives in TypeScript today.
**Acceptance criterion**: Field test "Mission with localization revoked mid-way" — Nav2 goal is cancelled, robot stops, operator gets an event log entry. No physical motion to phantom coordinates.
**Effort**: M.
**Prerequisites**: none.

---

## F. What this Phase 0 does NOT cover (deferred to Phase N)

| Topic | Deferred to | Why deferred |
|---|---|---|
| Build reproducibility timings (<5 min Orin) | Phase 1 (`01_foundations.md`) | Requires hardware |
| TF jitter / extrapolation | Phase 1 | Requires running stack |
| URDF inertias (1.0 placeholder) | Phase 2 (`02_hal.md`) | Linked to ros2_control discussion |
| ZED depth FPS sustained | Phase 2 | Hardware |
| Calibration UMBmark execution | Phase 3 | Hardware |
| AprilTag-as-factor vs `SetPose` analysis | Phase 4 | Architectural — needs file-level read |
| Costmap inflation saturation in 1.0–1.5 m corridors | Phase 7 | Needs experiment + nav2 params analysis |
| HAZOP / FMEA | Phase 9 | Significant work; create skeleton |
| GUI usability test with non-ROS technician | Phase 11 | Needs a real technician |
| 8 h × 5 day endurance MTBF | Phase 15 | Field, multi-day |

---

## G. Reader's checklist (the "< 30 min" requirement)

A new engineer who reads this file should be able to answer:

- [x] How many ROS 2 packages and what languages? **24 packages; mostly C++17, some TypeScript (UI backend), some Python (HIL bridges + dev tools).**
- [x] What is the production entry point? **`agv_start.sh` → `agv_full.launch.py` with `AGV_MODE=real`.**
- [x] How does the robot pick which map to load on boot? **`AGV_MAP` env > `~/.agv/last_map` > `default_empty.yaml` fallback.**
- [x] Who owns `map→odom`? `odom→base_link`? **`ekf_global` and `ekf_local` respectively. Single publishers.**
- [x] What runtime modes exist? **Four-layer matrix: systemd, has_map, currentMode, RobotState. See [`specs/state_machine.yaml`](../../../specs/state_machine.yaml).**
- [x] Where do maps live? **`${AGV_DATA_DIR}/maps/<X>.{yaml,pgm,area}` + `<X>_cuvslam/` + `<X>_meta.json`.**
- [x] What is the cmd_vel chain? **See B.3.**
- [x] What stops the robot? **Software path: `cmd_vel_gate` → zero output on `safety_status.safety_ok=false` OR `/agv/e_stop=true`. Hardware path: planned via `/agv/hardware_estop`, not yet wired (see `specs/interfaces.yaml#topics./agv/hardware_estop.status=planned`).**
- [x] What are the **biggest** unsolved problems? **CR-00-01 (repo not self-contained), CR-00-02 (geometry SSOT broken), CR-00-06 (waypoint bypass).**

---

## H. Status (end of Phase 0)

| Item | Status |
|---|---|
| Workspace cartography | ✅ complete |
| Critical findings filed | ✅ 6 (CR-00-01 … CR-00-06) |
| Spec verifier baseline | ✅ green |
| Drift from 2026-04-13 audit | ✅ none observed (all flagged bugs are `FIXED` or whitelisted) |
| Hardware-dependent gates | ⏸ deferred (documented in F) |
| Phase 1 file | ⏳ next |

End of Phase 0.
