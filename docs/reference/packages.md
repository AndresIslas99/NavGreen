# Package reference

NavGreen is a colcon workspace with **25 first-party ROS 2 packages in
[`src/`](https://github.com/AndresIslas99/NavGreen/tree/main/src)**, plus
the optional fleet layer in
[`fleet/`](https://github.com/AndresIslas99/NavGreen/tree/main/fleet) and
the operator dashboard in
[`web/`](https://github.com/AndresIslas99/NavGreen/tree/main/web).
Package names keep the historical `agv_` prefix (like Nav2's `nav2_*` packages
live under the Nav2 brand).

Every package carries its own `CLAUDE.md` contract — responsibilities, owned
and consumed interfaces, invariants, failure modes. Read it before touching a
package; it doubles as the package's documentation. The interface claims below
are cross-referenced against
[`specs/interfaces.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/interfaces.yaml)
(see [Interfaces & specs](interfaces.md)).

!!! note "How to read the CI column"
    CI ([`.github/workflows/ci.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/.github/workflows/ci.yaml))
    builds 20 of the 25 `src/` packages with `-Werror` in the `build-and-test`
    job and runs `colcon test` on 15 of them. `agv_sim` is built and
    smoke-tested in a dedicated `simulation` job. Four packages are excluded
    because their compile-time dependencies are vendor SDKs not on public apt:
    `agv_map_manager` (NVIDIA Isaac ROS), `agv_localization_init` (ZED
    `zed_msgs`), `agv_factor_graph` (GTSAM), and `agv_bringup` (which declares
    all three as runtime dependencies). TypeScript packages build (and, where
    tests exist, test) in the `typescript-build` job.

## Drivetrain & safety chain

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_interfaces`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_interfaces) | Custom ROS 2 definitions: 4 messages (`Mission`, `Waypoint`, `SafetyStatus`, `RailStartPoint`) and 8 services (missions, waypoints, map save/load, zones, rail ops) | `agv_interfaces/msg/*`, `agv_interfaces/srv/*` | Built + tested in CI |
| [`agv_odrive`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_odrive) | Production ODrive S1 CAN driver (250 kbps) with 50 Hz differential-drive wheel odometry, CAN feedback-loss watchdog, thermal protection | Pub: `/agv/wheel_odom`, `/agv/motor_state`, `/agv/drive_debug`. Sub: `/agv/cmd_vel_safe` (or `/agv/cmd_vel` in map-less mode), `/agv/e_stop`, `/agv/motor_enable` | Built + tested in CI. Also ships Python commissioning/diagnostic tools |
| [`agv_hw_interface`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_hw_interface) | `ros2_control` SystemInterface plugin wrapping the same CAN protocol, so `diff_drive_controller` or mock components can drive the robot | `agv_ros2control_mock.launch.py` — the lightest no-hardware entry point | Built + tested in CI; opt-in alternative to `agv_odrive`. The mock launch is a blocking CI smoke test |
| [`agv_safety`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_safety) | Software safety supervisor + `cmd_vel` gate — the last software element before the motor driver. Operational safeguard, **not** certified functional safety | Pub: `/agv/safety/status`, `/agv/cmd_vel_safe`. Sub: `/agv/cmd_vel_collision_safe`; latched `transient_local` subscriptions for the planned `/agv/hardware_estop` and `/agv/software_estop` | Built + tested in CI. Gate starts blocked until the supervisor proves liveness |
| [`agv_description`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_description) | URDF/Xacro robot model, TF chain, sensor mounts; nominal geometry in `config/robot_params.yaml` | `robot_state_publisher` owns `base_link → children` TF | Built in CI (also in the `simulation` job) |
| [`agv_bringup`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_bringup) | Launch orchestration — owns the three entry points: `agv_full.launch.py`, `agv_mapping.launch.py`, `agv_hil_full.launch.py` | Startup DAG specified in [`specs/launch_sequence.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/launch_sequence.yaml) | **Not built in CI** — declares all three vendor-SDK packages as runtime deps. The full stack also requires the external `agv_slam` (cuVSLAM) overlay, which is not published |

## Localization & perception

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_sensor_fusion`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_sensor_fusion) | Dual EKF (`robot_localization`): `ekf_local` owns `odom→base_link`, `ekf_global` owns `map→odom`. Also the wheel-slip detector, caster dwell advisor, IMU filter, and fusion monitor | Pub: `/agv/odometry/local`, `/agv/odometry/global`, `/agv/pose`, `/agv/wheel_odom_validated`, `/agv/imu/filtered`. Srv: `/agv/set_pose` (served by `ekf_global` only) | Built + tested in CI. ROS-free logic cores (slip detector, dwell advisor) have gtest coverage |
| [`agv_factor_graph`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_factor_graph) | GTSAM iSAM2 sliding-window estimator, validation-parallel to `ekf_global` | Consumes fused odometry streams | **Vendor SDK (GTSAM) — not built in CI**, no tests yet ([#12](https://github.com/AndresIslas99/NavGreen/issues/12)). Known caveat: it currently consumes `ekf_global` output, so it is not an independent check ([#10](https://github.com/AndresIslas99/NavGreen/issues/10)) |
| [`agv_localization_init`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_localization_init) | Auto-initialization orchestrator: localizes against a pre-built cuVSLAM keyframe DB when a map loads (Path A0/A/B/C cascade, AprilTag pose hints) — no manual "click to set pose" | Pub: `/agv/localization/state`. Srv: `/agv/localization/reinitialize`, `/agv/localization/save_last_known_pose` | **Vendor SDK (`zed_msgs`, ZED ROS 2 wrapper) — not built in CI**, no tests yet ([#12](https://github.com/AndresIslas99/NavGreen/issues/12)) |
| [`agv_scan_mapper`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_scan_mapper) | Live 2D occupancy grid from `LaserScan` (Bayesian log-odds, Bresenham raycasting) for commissioning and the dashboard map view | Pub: `/agv/live_map` (`transient_local`). Sub: a `LaserScan` input topic | Built + tested in CI |
| [`agv_markers`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_markers) | AprilTag (tag36h11) pose correction: feeds absolute pose corrections to `ekf_global` and hard-resets it on relocalization | Pub: `/agv/marker_pose`, `/agv/marker_raw_detected`. Calls `/agv/set_pose`. Hot-reloads the tag registry on `/agv/markers/registry_reload` | Built + tested in CI |
| [`agv_zone_detector`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_zone_detector) | Classifies the robot pose into greenhouse zones (corridor vs rail aisle) so Nav2 is never used inside rail aisles | Pub: `/agv/zone/state` (JSON, 10 Hz) | Built + tested in CI |

## Navigation & missions

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_navigation`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_navigation) | Nav2 configuration and launch: planning, MPPI control, collision monitor | Nav2 output on `/agv/cmd_vel_nav` (Phase 2 launches) or `/agv/cmd_vel` (legacy); collision monitor emits `/agv/cmd_vel_collision_safe` | Built in CI |
| [`agv_mode_arbiter`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_mode_arbiter) | 8-state FSM that **owns `/agv/cmd_vel`**: selects exactly one upstream controller (corridor nav ↔ rail approach ↔ rail drive ↔ teleop ↔ idle) and relays its Twist | Pub: `/agv/cmd_vel`, `/agv/mode/state`, `/agv/rail_driver/goal`. Sub: `/agv/cmd_vel_nav`, `/agv/cmd_vel_approach`, `/agv/cmd_vel_rail`, `/agv/zone/state`, `/agv/mode/set` | Built + tested in CI. Pure FSM core — no control math |
| [`agv_behaviors`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_behaviors) | BehaviorTree.CPP mission execution | Behavior trees under `trees/` | Built + tested in CI. One of two mission-execution paths — consolidation tracked in [#7](https://github.com/AndresIslas99/NavGreen/issues/7) |
| [`agv_waypoint_manager`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_waypoint_manager) | Mission CRUD (storage) + sequential waypoint dispatch | Calls `/agv/navigate_to_pose` directly (documented `known_gap`: bypasses the dashboard's dispatch gates) | Built + tested in CI. See [#7](https://github.com/AndresIslas99/NavGreen/issues/7) |
| [`agv_map_manager`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_map_manager) | Map persistence (with cuVSLAM + ZED area-memory sidecars), keepout/speed zones | Srv: `/agv/map_manager/save_map`, `load_map`, `update_zone`. Pub: `/agv/maps/loaded`, `/agv/current_map` (`transient_local`) | **Vendor SDK (Isaac ROS interfaces) — not built in CI** |

## Rail operation (Phase 2)

Driving on greenhouse heating-pipe rails.

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_rail_approach`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_rail_approach) | Precision AprilTag-guided approach to rail-start points (coarse Nav2 approach + fine visual servoing) | Pub: `/agv/cmd_vel_approach`, `/agv/rail_approach/state`. Srv: `/agv/rail_approach/execute`, `abort`, `list_rail_starts` | Built + tested in CI |
| [`agv_rail_detector`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_rail_detector) | Rail tube pair detection from ZED depth (BEV projection + RANSAC) for visual lateral correction | Pub: `/agv/rail_detections` (PoseArray, 5 Hz), `/agv/rail_detector/state` | Built + tested in CI — compiles without the ZED SDK, but needs the camera at runtime |
| [`agv_rail_driver`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_rail_driver) | Longitudinal-only drive along the rails — `angular.z` is hard-coded to 0 (any rotation risks hitting a rail tube) | Pub: `/agv/cmd_vel_rail`, `/agv/rail_driver/state`. Srv: `/agv/rail_driver/cancel_goal` | Built + tested in CI. ROS-free controller core with gtest coverage. Typed collision-monitor subscription tracked in [#16](https://github.com/AndresIslas99/NavGreen/issues/16) |

## Operator interface

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_image_server`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_image_server) | C++ MJPEG HTTP server for camera and depth-heatmap streams | HTTP `:8091` — `/camera/stream`, `/camera/snapshot`, `/depth/stream`, `/depth/snapshot` | Built + tested in CI. Plain HTTP, no auth — see [Security](../community/security.md) |
| [`agv_ui_backend`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_ui_backend) | Operator backend: REST + WebSocket bridge between the dashboard and the ROS 2 graph (teleop, missions, maps, nav goals, e-stop, motor arming) | HTTP/WS `:8090` (contract: [`specs/hmi_api.yaml`](https://github.com/AndresIslas99/NavGreen/blob/main/specs/hmi_api.yaml)). Pub: `/agv/e_stop`, `/agv/motor_enable`, `/agv/mode`, `/agv/mode/set`. Calls `/agv/navigate_to_pose` behind dispatch gates | TypeScript (Express + rclnodejs) — the one non-C++ runtime component, it is not a robot *control* node. Built in CI; unit tests tracked in [#11](https://github.com/AndresIslas99/NavGreen/issues/11) |
| [`web/agv_dashboard`](https://github.com/AndresIslas99/NavGreen/tree/main/web/agv_dashboard) | React operator dashboard (ISA-101 HMI style): teleop, live map, missions, safety pills | Talks to `agv_ui_backend` on `:8090` and `agv_image_server` on `:8091` | TypeScript/Vite. CI runs `tsc --noEmit`, vitest, and the production build |

## Development & testing

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`agv_hil_bridges`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_hil_bridges) | Hardware-in-the-loop bridge nodes: integrate sim `/agv/joint_states` into `/agv/wheel_odom` with the production kinematic parameters; visual-SLAM fallback relay | HIL-only overrides declared as `hil_override` in `specs/interfaces.yaml` | `dev_only: true` (Python permitted) — never loaded in production. Built in CI |
| [`agv_sim`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_sim) | Gazebo Classic simulation — spawn the AGV with no hardware, drive it via `/cmd_vel`, watch `/odom`. Reuses the real robot geometry and production `diff_drive_controller` gains | `ros2 launch agv_sim teleop_sim.launch.py` / `sim.launch.py` | `dev_only: true`. **Drivetrain-only: no cameras or lidar yet** — sensors + Nav2 in sim are roadmap work ([#6](https://github.com/AndresIslas99/NavGreen/issues/6)). CI builds it, validates the URDF, and runs headless Gazebo as best-effort ([#20](https://github.com/AndresIslas99/NavGreen/issues/20)) |
| [`agv_integration_tests`](https://github.com/AndresIslas99/NavGreen/tree/main/src/agv_integration_tests) | System-level integration tests in three tiers: ROS-free oracle tests (always run), stack-required tests (`AGV_STACK_TEST=1`), HIL tests | Exercises e-stop propagation, waypoint precision, EKF frames against a running stack | `dev_only: true`. Built in CI (not in CI's `colcon test` selection). The ROS-free oracle tier runs anywhere; stack tests skip explicitly unless `AGV_STACK_TEST=1` |

## Fleet layer (optional)

VDA 5050 multi-robot layer — **not** part of the default robot runtime. Started
explicitly via `fleet/start_fleet.sh` or the systemd units. See
[`fleet/README.md`](https://github.com/AndresIslas99/NavGreen/blob/main/fleet/README.md).

| Package | Purpose | Key interfaces | Status / notes |
|---|---|---|---|
| [`fleet/agv_fleet_manager`](https://github.com/AndresIslas99/NavGreen/tree/main/fleet/agv_fleet_manager) | VDA 5050 master: fleet state aggregation, order dispatch, traffic zones | REST + WebSocket on `:8092` (`FLEET_PORT`); token auth via `FLEET_API_TOKEN` | TypeScript. Built + vitest in CI |
| [`fleet/agv_vda5050_adapter`](https://github.com/AndresIslas99/NavGreen/tree/main/fleet/agv_vda5050_adapter) | Per-robot bridge: ROS 2 graph ↔ VDA 5050 MQTT | Calls `/agv/navigate_to_pose`; publishes `/agv/e_stop` for the `emergencyStop` / `clearEmergencyStop` instant actions (documented `known_gap`: bypasses the dashboard's dispatch gates) | TypeScript (rclnodejs). Built in CI; tests tracked in [#11](https://github.com/AndresIslas99/NavGreen/issues/11) |
| [`fleet/mosquitto`](https://github.com/AndresIslas99/NavGreen/tree/main/fleet/mosquitto) | MQTT broker configuration (VDA 5050 backbone, ports 1883/9001) | Ships `allow_anonymous true`; enable procedure documented in `mosquitto.conf` | Config only — read [Security](../community/security.md) before deploying |
| [`fleet/systemd`](https://github.com/AndresIslas99/NavGreen/tree/main/fleet/systemd) | systemd service units for broker, fleet manager, and adapter | `agv-mosquitto.service`, `agv-fleet-manager.service`, `agv-vda5050-adapter.service` | Config only |

## What you can run without the robot

- **`agv_sim`** — Gazebo physics, drivetrain only (no sensors). Start here:
  [Drive the robot in simulation](../tutorials/drive-in-simulation.md).
- **`agv_hw_interface` mock launch** — the `ros2_control` stack with mock
  components, no Gazebo at all.
- **The TypeScript stack** — dashboard, UI backend, fleet layer all build and
  run without hardware (the backend needs a sourced ROS 2 environment for
  rclnodejs).

!!! warning "The full autonomy stack needs vendor SDKs and hardware"
    `agv_bringup`'s production launches require the unpublished `agv_slam`
    (cuVSLAM) overlay plus the Isaac ROS / ZED / GTSAM vendor stacks, and the
    HIL loop needs the maintainer's private simulation host. Making the full
    stack runnable by contributors is the top roadmap item — see the
    [Roadmap](../community/roadmap.md).
