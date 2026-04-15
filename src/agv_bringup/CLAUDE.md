# agv_bringup

Launch orchestration package for all AGV operational modes. Provides parameterized
launch files with sequenced startup to ensure correct initialization order.

**Canonical startup DAG lives in [specs/launch_sequence.yaml](../../specs/launch_sequence.yaml).**
That spec is the source of truth for preconditions, timings, and failure
modes. The table below is a human-readable summary that must stay in sync.

## Launch Files

Only 3 launch files ship. Older experimental files (`agv_modular`, `nav`,
`hardware`, `dashboard`, `perception`, `agv_teleop`, `agv_robot_core`,
`agv_fusion`, `agv_ekf_local_test`, `agv_hil`) were deleted during the
2026-04-13 audit (Fase 6 bug #4) — they were unreachable from `agv_start.sh`
and confused readers about which entry point was real. Git history preserves
them if they need to be revived.

| File | Purpose | AGV_MODE |
|------|---------|----------|
| `agv_full.launch.py` | **Production**: full autonomy stack with Nav2 + safety chain | `real` (default) |
| `agv_mapping.launch.py` | Commissioning mapping: cuVSLAM owns TF, SLAM Toolbox in mapping mode, no Nav2 | `mapping` |
| `agv_hil_full.launch.py` | Hardware-in-the-loop: full stack with simulated sensor inputs | `hil` |

## Full Stack Startup Sequence (agv_full.launch.py)

See [specs/launch_sequence.yaml](../../specs/launch_sequence.yaml) for the
authoritative DAG with preconditions and failure modes. Summary:

| Delay | Component | Package | Condition |
|-------|-----------|---------|-----------|
| 0s | robot_state_publisher (via description.launch.py) | agv_description | always |
| 0s | odrive_can_node | agv_odrive | always (cmd_vel topic remap depends on has_map) |
| 0s | pointcloud_to_laserscan (FOV ±90°, min_height 0.01m) | external | always |
| 0s | image_server (port 8091, MJPEG camera+depth) | agv_image_server | always |
| 0s | scan_grid_mapper (live occupancy grid, 0.025m res) | agv_scan_mapper | always |
| 3s | cuVSLAM + nvblox (TF DISABLED via cuvslam_greenhouse.yaml /**: key) | agv_slam | always |
| 3.5s | imu_filter (Butterworth vibration filter) | agv_sensor_fusion | always |
| 4s | ekf_local + ekf_global + fusion_monitor | agv_sensor_fusion | always |
| 4.5s | factor_graph (parallel, publish_tf=false) | agv_factor_graph | always |
| 5s | slam_toolbox_localization (optional, TF DISABLED) | slam_toolbox | enable_slam_localization |
| 5s | map_manager + waypoint_manager | agv_map_manager, agv_waypoint_manager | always |
| 6s | Nav2 stack (SmacPlanner2D + MPPI + collision_monitor) | agv_navigation | `has_map` |
| 6.5s | agv_safety (safety_supervisor + cmd_vel_gate) | agv_safety | `has_map` |
| 7s | AprilTag + marker_correction + rail_approach + auto_init_orchestrator | multiple | enable_markers |
| 8s | teleop_server (dashboard on :8090) | agv_ui_backend | always |

## Common Launch Arguments

| Argument | Default | Used In |
|----------|---------|---------|
| `namespace` | `"agv"` | All launch files |
| `use_sim_time` | `"false"` | All launch files |
| `map` | (required for nav) | agv_full, agv_navigation |
| `enable_markers` | `true` | agv_full |
| `enable_behaviors` | `false` | agv_full |
| `enable_slam_localization` | `true` | agv_full |

## TF Ownership (Critical)

- `odom -> base_link`: ekf_local ONLY (agv_sensor_fusion)
- `map -> odom`: ekf_global ONLY (agv_sensor_fusion)
- `base_link -> wheels`: robot_state_publisher (agv_description)
- cuVSLAM: **topic-only** — TF disabled via `cuvslam_greenhouse.yaml` (`/**:` key, not node name)
- SLAM Toolbox: **topic-only** — TF disabled via `transform_publish_period: 0.0`
- ZED wrapper: `publish_tf: false`, `publish_imu_tf: true` (IMU calibration TF only)

**WARNING**: YAML override files MUST use `/**:` as the parameter key, not the node name. The node name in ROS2 may differ from the YAML key. Using the wrong key silently fails to apply parameters.

## Configuration

- `config/cyclonedds_hil.xml` — DDS config for HIL mode
- `config/cuvslam_no_tf.yaml` — cuVSLAM TF disable override (`/**:` key)
- `config/cuvslam_greenhouse.yaml` — cuVSLAM greenhouse tuning + TF disable (`/**:` key)

## Dependencies

- All src/ packages, pointcloud_to_laserscan, isaac_ros_visual_slam

## Improvement Opportunities

- Document all available launch arguments per file (currently discoverable only by reading code)
- Add launch argument validation (e.g., map file existence check)
- Add a launch health monitor that verifies expected nodes are running after startup
- Document recommended launch file selection for each operational scenario
