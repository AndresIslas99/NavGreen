# agv_bringup

Launch orchestration package for all AGV operational modes. Provides parameterized
launch files with sequenced startup to ensure correct initialization order.

## Launch Files

| File | Purpose | Delay Sequence |
|------|---------|---------------|
| `agv_full.launch.py` | **Production**: Full autonomy stack | 0-7s sequenced |
| `agv_robot_core.launch.py` | Minimal: URDF + motor control | Immediate |
| `agv_teleop.launch.py` | Teleoperation mode only | Immediate |
| `agv_mapping.launch.py` | SLAM + odometry for map building | 0-2s |
| `agv_navigation.launch.py` | Nav2 + mission execution | 0-6s |
| `agv_fusion.launch.py` | Dual EKF only | 0-4s |
| `agv_ekf_local_test.launch.py` | Local EKF testing | Immediate |
| `agv_hil.launch.py` | Hardware-in-the-loop simulation | Immediate |
| `agv_hil_full.launch.py` | Full stack in HIL mode | 0-7s |

## Full Stack Startup Sequence (agv_full.launch.py)

| Delay | Component | Package |
|-------|-----------|---------|
| 0s | robot_state_publisher | agv_description |
| 0s | odrive_can_node | agv_odrive |
| 0s | pointcloud_to_laserscan | external |
| 0s | teleop_server | agv_ui_backend |
| 2s | cuVSLAM | agv_slam (external) |
| 4s | ekf_local + ekf_global | agv_sensor_fusion |
| 5s | map_manager + waypoint_manager | agv_map_manager, agv_waypoint_manager |
| 6s | Nav2 stack | agv_navigation |
| 7s | AprilTag detection (optional) | agv_markers |
| 7s | Behavior executor (optional) | agv_behaviors |

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

- `odom -> base_link`: ekf_local (agv_sensor_fusion)
- `map -> odom`: ekf_global (agv_sensor_fusion)
- `base_link -> wheels`: robot_state_publisher (agv_description)
- cuVSLAM: **topic-only**, does NOT publish TF

## Configuration

- `config/cyclonedds_hil.xml` — DDS config for HIL mode
- `config/cuvslam_no_tf.yaml` — cuVSLAM parameters with TF publishing disabled

## Dependencies

- All src/ packages, pointcloud_to_laserscan, isaac_ros_visual_slam

## Improvement Opportunities

- Document all available launch arguments per file (currently discoverable only by reading code)
- Add launch argument validation (e.g., map file existence check)
- Add a launch health monitor that verifies expected nodes are running after startup
- Document recommended launch file selection for each operational scenario
