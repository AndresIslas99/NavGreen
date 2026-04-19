# Architectural Gaps and Adoption Roadmap

> **This is a planning document, not a spec.** Binding decisions live in [specs/](../specs/) and [policies/engineering_rules.md](../policies/engineering_rules.md). This file describes where the workspace structure should evolve and in what order. It does not override [CLAUDE.md](../CLAUDE.md) or any TASK.yaml.

## Status

- **Verified**: 2026-04-11
- **Against commit**: `a8180b7` (main)
- **Verified by**: direct grep / file reads on the working tree (see each gap for the exact check).

The current system topology is captured in [docs/architecture.md](architecture.md). Per-package build and test status is in [docs/production_readiness_assessment.md](production_readiness_assessment.md). This document is the forward-looking complement to those two — what is missing and how to close it.

---

## Gap 1 — ros2_control migration

### Current state
[src/agv_odrive/](../src/agv_odrive/) is a standalone ROS2 node that talks to the ODrive over CAN, publishes wheel odometry, and subscribes to `/cmd_vel`. There are zero references to `ros2_control`, `controller_manager`, or `hardware_interface` in any `package.xml` under [src/](../src/) (verified via grep against `src/**/package.xml`).

### Target state
Wrap the existing CAN code as a `hardware_interface::SystemInterface` plugin loaded by `controller_manager`. Use `diff_drive_controller` from `ros2_controllers` for kinematics and odometry. Add a `mock_components/GenericSystem` configuration so navigation and behaviors can run end-to-end without the physical robot.

### Why it matters
Without `ros2_control`, anyone working on navigation, behaviors, or the dashboard either needs the real robot or has to mock topics by hand. Both block parallel development. `mock_components` is a free deliverable of the migration — no extra code needed.

### Adoption cost
Several days. Touches [src/agv_odrive/](../src/agv_odrive/), [src/agv_description/](../src/agv_description/) (URDF needs `<ros2_control>` blocks), and [src/agv_bringup/](../src/agv_bringup/) (launch + controller config). The CAN code itself is wrapped, not rewritten — `read()` and `write()` become method bodies on the plugin class.

### Risk if deferred
Development velocity stays bottlenecked on hardware availability. Every new contributor has to learn the bespoke topic shape of `agv_odrive` instead of the standard `diff_drive_controller` interface.

---

## Gap 2 — agv_safety package

### Current state
No `src/agv_safety/` directory exists (verified via glob). Collision monitoring lives only as a Nav2 component configured in [src/agv_navigation/config/collision_monitor.yaml](../src/agv_navigation/config/collision_monitor.yaml). There is no central supervisor for node heartbeats, software E-stop arbitration, velocity limiting, or ODrive thermal monitoring.

### Target state
A new package `agv_safety` containing:
- A `SafetyStatus.msg` in [src/agv_interfaces/](../src/agv_interfaces/) (or in `agv_safety/msg/` if the team prefers per-package messages).
- A C++17 `safety_supervisor` node that monitors deadline QoS on critical topics, aggregates per-node heartbeats, and publishes a single `safety_ok` boolean plus a structured status.
- A `cmd_vel` mux/gate node that only forwards velocity commands when `safety_ok` is true.
- Zero internal dependencies beyond `agv_interfaces`.

This complements but does **not** replace the operational guardrails listed in [Rule 6 of policies/engineering_rules.md](../policies/engineering_rules.md). Software safety remains operational only — certified functional safety stays out of scope.

### Why it matters
Greenhouse operation in Mexico places the AGV near workers. Even without certified hardware safety, a software supervisor that forces a stop when any critical node goes silent is a baseline operational requirement. Today, if `agv_sensor_fusion` crashes mid-run, nothing forces the motors to stop until Nav2's collision monitor reacts to a downstream symptom.

### Adoption cost
A stub (empty package + msg + node skeleton + launch_testing) is half a day. The full supervisor with deadline-QoS wiring and a real velocity gate is a few days, partly because it has to be integrated into [src/agv_bringup/launch/agv_full.launch.py](../src/agv_bringup/launch/agv_full.launch.py) before any other node can publish to `/cmd_vel`.

### Risk if deferred
A single-node failure in the perception or fusion stack does not automatically translate into a stop command. The current Nav2 collision monitor catches some failure modes but is not a watchdog over the rest of the pipeline.

---

## Gap 3 — Centralized configuration layout

### Current state
[src/agv_bringup/config/](../src/agv_bringup/config/) exists but contains only four files: `cuvslam_greenhouse.yaml`, `cuvslam_no_tf.yaml`, `cyclonedds_hil.xml`, `cyclonedds_production.xml`. The rest of YAML configuration lives inside individual packages (`agv_navigation/config/`, `agv_sensor_fusion/config/`, etc.). There is no Clearpath-style `common/ robot/ greenhouse/ simulation/` hierarchy.

### Target state
Move (or symlink, then migrate) shared YAML into a centralized layout under `agv_bringup/config/`:

```
src/agv_bringup/config/
  common/         # parameters shared across deployments
  robot/          # this AGV's physical config
  greenhouse/     # site-specific (chada/, future sites)
  simulation/     # sim-only overrides
  hil/            # hardware-in-the-loop overrides
```

A launch argument `site:=<name>` (default `chada`) selects which `greenhouse/<name>/` overlay to merge on top of `common/` and `robot/`. Per-package configs still exist for things that are genuinely package-internal — the goal is centralization of *deployment* parameters, not all parameters.

### Why it matters
Deploying to a second greenhouse currently requires editing YAML files inside multiple packages. With the overlay layout, an integrator changes a single launch argument. This is also the precondition for any future multi-site deployment.

### Adoption cost
One afternoon, organizational only. Risk is mostly that a launch file misses an updated path during the move — easy to catch with `colcon test` and a smoke run.

### Risk if deferred
Each new site or robot variant duplicates YAML across packages, and drift between packages becomes inevitable.

---

## Gap 4 — Per-package launch_testing and coverage parity

### Current state
**This gap was reformulated during verification.** Unit tests already exist across nine packages (verified via glob `src/*/test/test_*.cpp`):

- [src/agv_odrive/test/test_kinematics.cpp](../src/agv_odrive/test/test_kinematics.cpp), [test_odom_integration.cpp](../src/agv_odrive/test/test_odom_integration.cpp)
- [src/agv_slam/test/test_slam_monitor_node.cpp](../src/agv_slam/test/test_slam_monitor_node.cpp), [test_rate_tracker.cpp](../src/agv_slam/test/test_rate_tracker.cpp), [test_tegrastats_parser.cpp](../src/agv_slam/test/test_tegrastats_parser.cpp)
- [src/agv_map_manager/test/test_zone_persistence.cpp](../src/agv_map_manager/test/test_zone_persistence.cpp)
- [src/agv_waypoint_manager/test/test_mission_persistence.cpp](../src/agv_waypoint_manager/test/test_mission_persistence.cpp)
- [src/agv_markers/test/test_marker_lookup.cpp](../src/agv_markers/test/test_marker_lookup.cpp)
- [src/agv_behaviors/test/test_bt_loading.cpp](../src/agv_behaviors/test/test_bt_loading.cpp)
- [src/agv_scan_mapper/test/test_grid_math.cpp](../src/agv_scan_mapper/test/test_grid_math.cpp)
- [src/agv_sensor_fusion/test/test_sensor_health.cpp](../src/agv_sensor_fusion/test/test_sensor_health.cpp)
- [src/agv_image_server/test/test_image_pipeline.cpp](../src/agv_image_server/test/test_image_pipeline.cpp)

The unit-test layer is real and not the gap. The actual gaps are:

1. **Coverage is uneven** — some packages test pure kinematics thoroughly, others have one smoke test.
2. **No `launch_testing` per package** — there is no `<package>/test/launch/test_<name>.launch.py` pattern. Integration coverage is concentrated in [src/agv_integration_tests/](../src/agv_integration_tests/) at the system level only, which means a single broken node fails an opaque end-to-end test instead of a targeted package-level test.

### Target state
- Each package adds one `launch_testing` test that brings up just that package's nodes (with `mock_components` from Gap 1, where applicable) and asserts on its public topics/services.
- Coverage parity is enforced by adding a test count to the [docs/production_readiness_assessment.md](production_readiness_assessment.md) matrix and reviewing it during package work.
- [src/agv_integration_tests/](../src/agv_integration_tests/) stays for true end-to-end flows.

### Why it matters
A developer touching one package can run their tests in seconds without launching the full stack. End-to-end tests stay valuable but stop being the only signal.

### Adoption cost
Adopt gradually as packages are touched, not as one big PR. Each package is one to two hours of `launch_testing` boilerplate plus a couple of assertions.

### Risk if deferred
The feedback loop stays slow for solo package work, and end-to-end test failures keep being hard to localize.

---

## Gap 5 — Perception composable container (NITROS)

### Current state
[src/agv_bringup/launch/agv_full.launch.py](../src/agv_bringup/launch/agv_full.launch.py) contains zero references to `ComposableNode`, `composable_node`, or `container` (verified via grep). cuVSLAM, `pointcloud_to_laserscan`, and `agv_image_server` all run as separate processes. On the AGX Orin this means GPU-to-GPU data is being copied through normal ROS2 transport instead of zero-copy via Isaac ROS NITROS.

### Target state
Group the GPU-bound perception nodes (cuVSLAM, image processing, any future crop-detection DNN) into a single `ComposableNodeContainer` declared in a new `perception.launch.py`. Split [agv_full.launch.py](../src/agv_bringup/launch/agv_full.launch.py) into three top-level launch files orchestrated by a single bringup file:

- `hardware.launch.py` — motors and direct sensors
- `perception.launch.py` — the composable container
- `navigation.launch.py` — Nav2 stack

This is a launch-file change. **No source code is rewritten.**

### Why it matters
On a Jetson, separating perception nodes into discrete processes leaves zero-copy GPU performance on the table. NITROS-style composition is the standard Isaac ROS pattern and is what the AGX Orin was chosen to enable.

### Adoption cost
Half a day to a day. Risk is low because nodes that are designed as components can be loaded into a container without code changes; the only failure mode is finding a node that was not built as a component, in which case it stays a process.

### Risk if deferred
Perception latency and GPU memory traffic stay higher than necessary. This is invisible until the system is stressed under real greenhouse field conditions, at which point it is harder to diagnose.

---

## Gap 6 — interfaces.yaml change governance

### Current state
[specs/interfaces.yaml](../specs/interfaces.yaml) is the canonical contract for TF frames, topics, services, and message rates. [policies/engineering_rules.md](../policies/engineering_rules.md) has eight rules but **none of them mentions `interfaces.yaml`** or how changes to it should be reviewed. The contract exists; the change process does not.

### Target state
Add a new rule (Rule 9) to [policies/engineering_rules.md](../policies/engineering_rules.md):

> **Rule 9 — Interface change governance.** Any change to [specs/interfaces.yaml](../specs/interfaces.yaml) requires review by every team that consumes or produces the affected topic, service, or frame. The PR description must list which packages depend on the change and why the new shape is necessary. Bump the version field at the top of `interfaces.yaml` on every merge.

This is a process change, not a code change. It is a separate small PR — not part of this roadmap doc.

### Why it matters
Today there is no friction against silently changing a topic name or rate, which would break consumers in other packages. The contract is only useful if changes to it are visible to the people who depend on it.

### Adoption cost
One small PR to `engineering_rules.md`.

### Risk if deferred
Drift between [specs/interfaces.yaml](../specs/interfaces.yaml) and actual runtime behavior, with no mechanism to catch it at review time.

---

## Adoption priority and sequencing

Ordered by impact-vs-cost. This ordering matches the user's analysis on 2026-04-11.

| # | Gap | Impact | Cost | When |
|---|---|---|---|---|
| 1 | ros2_control migration | High — unblocks parallel development | Several days | Before next major navigation work |
| 2 | agv_safety package | High — production prerequisite | Half day stub, days for full | Before first greenhouse field visit with personnel present |
| 3 | Config centralization | Medium — unblocks multi-site | One afternoon | Anytime; low risk |
| 4 | Perception composable container | Medium — GPU performance | Half day to a day | After Gap 1 (so the container also covers any new perception controllers) |
| 5 | Per-package launch_testing | Medium — developer velocity | Gradual | Adopt as packages are touched |
| 6 | interfaces.yaml governance | Low effort, important | One PR | Anytime |

Gaps 3 and 6 can be done immediately and independently. Gaps 1, 2, 4 each warrant their own plan file before implementation. Gap 5 is a habit, not a project.

---

## Out of scope for this document

- Implementing any of the six gaps. Each becomes its own future task with its own plan.
- Modifying [policies/engineering_rules.md](../policies/engineering_rules.md) to add Rule 9. That is the separate PR described in Gap 6.
- Updating [CLAUDE.md](../CLAUDE.md)'s canonical source order. This document is a downstream reference, not a source of authority.
- Touching anything under [src/](../src/).

---

## Local patches against vendored third-party packages

Track every modification we make to upstream code that lives under [src/](../src/) but is *not* ours. Each entry must list the files touched, the rationale, and the upstream-reporting status so a future `git pull` of the vendor source can be reconciled.

### LP-1 — zed-ros2-wrapper Area Memory save (added 2026-04-12)

**Vendor**: [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) at [src/zed-ros2-wrapper/](../src/zed-ros2-wrapper/)

**Problem**: The wrapper as shipped accepts `pos_tracking.area_memory_db_path` but only uses it as the LOAD path at startup. It never calls `disablePositionalTracking(path)` or `saveAreaMap(path)` on shutdown, so the SDK landmark database is loaded into RAM, refined throughout the session, and discarded on exit. This makes Area Memory effectively useless for cross-session relocalisation — exactly the use case Stereolabs documents.

**Patch**: Three additive changes, all marked with `// AGV greenhouse local patch` and `// ────────── end AGV greenhouse local patch ──────────` markers so they are obvious in `git diff`. No upstream lines were deleted or rewritten.

| File | Change |
|---|---|
| [src/zed-ros2-wrapper/zed_components/src/zed_camera/include/zed_camera_component.hpp](../src/zed-ros2-wrapper/zed_components/src/zed_camera/include/zed_camera_component.hpp) | Declare `callback_saveAreaMemory()`, `mSaveAreaMemorySrv` member, and `mSrvSaveAreaMemoryName` constant |
| [src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp](../src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp) (`~ZedCamera`) | Call `mZed->disablePositionalTracking(mAreaMemoryDbPath)` and poll `getAreaExportState()` with a 15 s timeout before tearing down threads — autosave on clean shutdown |
| [src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp](../src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp) (`initServices`) | Register the new `~/save_area_memory` Trigger service |
| [src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp](../src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component.cpp) (new method) | Implement `callback_saveAreaMemory()` — calls SDK `saveAreaMap(path)` and polls `getAreaExportState()` synchronously so the caller knows when the file is on disk |

**Total diff**: ~95 lines, all additive, in two files. No new package dependencies (uses `std_srvs::srv::Trigger` which the wrapper already had transitively).

**Why we didn't fork or vendor differently**: The wrapper is already cloned into our `src/` tree (it is not a binary apt package). Vendoring it means we own its build outputs. Patching in place keeps the change visible to anyone reading `git status` on the repo and avoids a wrapper-of-wrapper indirection.

**Why a service and not just the destructor patch**: The orchestrator and the dashboard need a runtime trigger to save Area Memory after a successful mapping run, *before* the wrapper shuts down. The destructor patch is the safety net for unclean shutdowns; the service is the primary mechanism. Both use the same SDK function (`saveAreaMap` is the documented async call; `disablePositionalTracking(path)` calls it internally per [Camera.hpp:10715](file:///usr/local/zed/include/sl/Camera.hpp)).

**Why a Trigger and not a custom srv**: To avoid making the wrapper depend on `agv_interfaces`. The destination path is read from the existing `pos_tracking.area_memory_db_path` parameter, which the wrapper already declares. One source of truth.

**Consumers (in this workspace) that depend on the patch**:
- [src/agv_map_manager/src/map_manager_node.cpp](../src/agv_map_manager/src/map_manager_node.cpp) — calls `/zed/zed_node/save_area_memory` from `on_save_map()` after the cuVSLAM keyframe DB save
- [src/agv_localization_init/src/auto_init_orchestrator_node.cpp](../src/agv_localization_init/src/auto_init_orchestrator_node.cpp) — Path A0 subscribes to `/zed/zed_node/pose_with_covariance` (the wrapper publishes this when `pos_tracking_enabled: true`, which is the same flag that enables the saved Area Memory load — flipped in [src/agv_slam/config/zed2i_override.yaml](../src/agv_slam/config/zed2i_override.yaml))
- [src/agv_sensor_fusion/config/ekf_global.yaml](../src/agv_sensor_fusion/config/ekf_global.yaml) — `pose1` source consumes the same wrapper topic as a continuous absolute correction

**Upstream status**: Not yet reported. Candidate issue/PR title for stereolabs/zed-ros2-wrapper: *"Expose `saveAreaMap()` as a ROS service and call `disablePositionalTracking(path)` in the destructor when `area_memory_db_path` is set"*. Both changes are individually small and self-contained — file as two PRs.

**Reconciliation procedure** when pulling new upstream wrapper code:
1. `git pull` the wrapper subtree (or rebase the wrapper branch).
2. `git diff` against this commit to surface the four marked blocks.
3. If the upstream reorganized `~ZedCamera()` or `initServices()`, re-apply the marked blocks at their new locations — they are self-contained and have no cross-references except `mPosTrackingStarted`, `mAreaMemoryDbPath`, and `mZed`, all of which are existing wrapper members.
4. `colcon build --packages-select zed_components` and rerun the verification: enable Path A0 in the orchestrator, mask all AprilTags, mode-switch to `nav`, confirm `LOCALIZED` reaches the dashboard via the SDK Area Memory path.
