# Fase 7 — Pre-flight navigation tuning findings (2026-04-13)

This document captures the cross-cutting dependency analysis and external best-
practices review performed before applying the 5 navigation fixes of Fase 7.
It is the input that justified the order of application, the decision about
what to block vs allow, and the known gaps carried into Fase 8.

## Trigger

A live navigation trip between 17:26 and 17:29 on 2026-04-13 failed with the
following symptoms:

- Goal: (0.52, -1.38) from current location (-0.78, 1.30), distance 2.98 m.
- After 3 min 30 s the robot had advanced 0.67 m (~22%) and the operator cancelled.
- `bt_navigator` logged 328 "Passing new path to controller" events in the trip.
- `controller_server` spammed "Control loop missed its desired rate of 20.0000Hz".
- `bt_navigator: Timed out while waiting for action server to acknowledge goal request for smooth_path`
- Localization state `FAILED` throughout the trip (mapa I2 sin sidecars `.area` / `_cuvslam/` / `_meta.json`).
- CPU dominated by `slam_web_gui_node.py` (94%), `nvblox_node` (50% + 9.6 GB RAM), `slam_container` (104%).

Four candidate fixes emerged from live inspection; the user asked to validate
cross-cutting impact and compare against external best practices before executing.

## Methodology

Three Explore agents ran in parallel, each with a narrow focus:

1. **Internal dependency analysis** — grep the workspace for every consumer,
   publisher, launch reference, spec reference, and test reference of each fix
   target. File:line citations.
2. **Nav2 + external navigation stacks review** — WebFetch of Nav2 docs and
   GitHub issues on MPPI, control loop rate, SmoothPath usage, lifecycle
   manager. Comparison with Clearpath Husky, Isaac ROS Nova, Autoware.
3. **ZED SDK + Isaac ROS cuVSLAM + visual SLAM review** — WebFetch of StereoLabs
   Positional Tracking docs, Isaac ROS cuVSLAM repo, robot_localization ecosystem
   practices, and comparison with SLAM Toolbox / Cartographer / RTAB-Map for
   per-map persistence conventions.

## Key findings

### Internal dependencies

| Fix | Dependency surface | Risk |
|---|---|---|
| F1 remove slam_web_gui | 0 ROS consumers of its outputs. FastAPI :8080 isolated from dashboard :8090. | Low (launch-only change). |
| F2 remove nvblox | 1 ROS consumer: `slam_web_gui_node` subscribes to `/nvblox_node/back_projected_depth/zed_left_camera_optical_frame`. No other workspace node consumes nvblox outputs. `agv_navigation/CLAUDE.md` already declares `nvblox_layer` removed from both costmaps. | Medium — must apply F1 first or simultaneously. |
| F3 unify map_dir | Single authoritative change site: `agv_bringup/launch/agv_full.launch.py:53`. `auto_init_orchestrator` and `agv_ui_backend` already use AGV_DATA_DIR. `default_empty.yaml` is a template in install share and can stay there. | Low — existing maps remain in place. |
| F4 localization gate | **MAJOR**: `agv_waypoint_manager::waypoint_manager_node.cpp:109` creates a direct action client to `/agv/navigate_to_pose`, bypassing the backend entirely. The gate only covers dashboard goals. | Partial coverage accepted; mission executor refactor tracked for Fase 8. |

### State classification for F4 (localization gate)

- `UNKNOWN` — boot default, cascade not yet started. **Allow** (first goal window).
- `INITIALIZING` — cascade in progress. **Allow** (don't block while system is warming up).
- `LOCALIZED` — ideal state. **Allow**.
- `DEGRADED` — visual-only, no AprilTag anchor. **Allow** (operator may consciously accept drift risk).
- `FAILED` — all 4 cascade paths exhausted. **BLOCK**.

### Nav2 external best practices

- **SmoothPath within PipelineSequence is an anti-pattern**. Nav2 tutorial `https://docs.nav2.org/tutorials/docs/adding_smoother.html` and issue #4710 confirm that `PipelineSequence` re-ticks all children on RUNNING. In practice our `RateController hz=2.0` caps re-execution at 2 Hz, but each action call still pays the ACK round-trip cost; with smoother_server CPU-starved, the 2 Hz calls timeout.
- **MPPI on Jetson AGX Orin** — config (batch_size=1000, time_steps=32, controller_frequency=20 Hz) is within the community-tuned range. The root cause of missed rate is not MPPI itself but compute oversubscription from dev tools. Issues #5375 and #5712 report the same pattern.
- **Lifecycle `is_active` service gate** we implemented in Fase 6 bug #3 is the idiomatic Nav2 pattern.
- `xy_goal_tolerance: 0.15, yaw_goal_tolerance: 0.25` is slightly tighter than Clearpath Husky defaults but appropriate for greenhouse precision.
- `RateController hz=2.0` is at the upper edge of AMR community practice (1-2 Hz).

### Visual SLAM + ZED SDK external best practices

- **Dual-EKF pattern canonical**: ekf_local + ekf_global with cuVSLAM differential + AprilTag absolute matches `robot_localization` documented architecture.
- **File-swap of ZED Area Memory is a workaround not officially supported** by StereoLabs. Works because the wrapper re-reads `pos_tracking.area_memory_db_path` on `reset_pos_tracking` (our patch). Documented as implicit contract.
- **AprilTag registry stale-tag vulnerability**: if a tag is moved without updating `apriltags.json`, AprilTag covariance (~0.01 m²) will dominate over visual SLAM (~0.1 m²) in ekf_global, pulling the pose to an incorrect location. Tracked as Fase 8 improvement (plausibility filter).
- **Per-map 5-sidecar decomposition** (`.yaml`, `.pgm`, `_cuvslam/`, `.area`, `_meta.json`) is more granular than SLAM Toolbox / Cartographer / RTAB-Map (which use a single artifact). Not an anti-pattern — consequence of using two independent SLAM backends (cuVSLAM + ZED SDK). Requires atomic save/load contract; partially implemented.
- **Isaac ROS cuVSLAM issue #194** matches the incident: `localize_in_map` fails when the pose hint is far from ground truth. In our case `I2_cuvslam/` was missing entirely, so no `localize_in_map` was attempted and the cascade fell directly to FAILED.

## Accepted known gaps (out-of-scope for Fase 7)

1. `waypoint_manager` bypasses the F4 gate by calling Nav2 directly. Requires refactor.
2. `marker_correction_node` has no plausibility filter for stale AprilTag detections.
3. ZED Area Memory file-swap is a workaround, not an official StereoLabs API.
4. Map integrity check at boot (verify all 5 sidecars exist) not implemented.
5. ZED IMU calibration in greenhouse orientation has not been validated.
6. MPPI → RegulatedPurePursuit fallback if post-cleanup still missing rate.

## Order of application (decided)

1. **F1 + F2 + F5 together** — launch file cleanup + BT SmoothPath refactor. Rebuild `agv_slam` + `agv_navigation`.
2. **F3** — `map_dir` unification in `agv_bringup`. Rebuild `agv_bringup`.
3. **F4** — localization gate in `agv_ui_backend`. `npm run build`.
4. systemctl restart + verify_specs + healthcheck.
5. Test: cargar I2 → gate bloquea (porque I2 sin sidecars) → remapear → Save → reboot → Load → LOCALIZED → teleop → goal.

## References

- Nav2 MPPI docs: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- Nav2 SmoothPath tutorial: https://docs.nav2.org/tutorials/docs/adding_smoother.html
- Nav2 Lifecycle: https://docs.nav2.org/configuration/packages/configuring-lifecycle.html
- Nav2 issue #4710 (SmoothPath re-tick): https://github.com/ros-navigation/navigation2/issues/4710
- Nav2 issue #5375 (MPPI Jetson tuning): https://github.com/ros-navigation/navigation2/issues/5375
- Nav2 issue #5712 (control loop missed rate): https://github.com/ros-navigation/navigation2/issues/5712
- StereoLabs Positional Tracking: https://www.stereolabs.com/docs/positional-tracking
- StereoLabs Area Memory: https://www.stereolabs.com/docs/positional-tracking/area-memory
- Isaac ROS cuVSLAM repo: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html
- Isaac ROS cuVSLAM issue #194 (localize_in_map failure): https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/issues/194
- robot_localization docs: http://docs.ros.org/en/melodic/api/robot_localization/html/
