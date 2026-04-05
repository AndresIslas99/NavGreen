# agv_navigation

Nav2 autonomous navigation configuration. Provides path planning (SmacPlanner2D),
trajectory following (RegulatedPurePursuit), and two-stage collision monitoring.
No custom C++ nodes — this package is configuration and launch only.

## Nodes (all from Nav2)

- **bt_navigator** — Behavior tree navigation coordinator
- **planner_server** — SmacPlanner2D global path planning (5 Hz, max 2.0s)
- **controller_server** — RegulatedPurePursuit local trajectory following (20 Hz)
- **behavior_server** — Recovery behaviors: spin, backup, wait
- **velocity_smoother** — cmd_vel smoothing
- **collision_monitor** — Two-stage obstacle protection (stop + slowdown zones)
- **map_server** — Static map serving
- **lifecycle_manager** — Manages lifecycle of all Nav2 nodes

## Topics

**Subscribed:**
- `/agv/scan` (LaserScan) — Primary obstacle source
- `/zed/zed_node/point_cloud/cloud_registered` (PointCloud2) — Depth obstacle source
- `odometry/global` (Odometry) — Robot pose from dual EKF

**Published:**
- `cmd_vel_smoothed` (Twist) — Smoothed commands
- `cmd_vel_safe` (Twist) — Safe commands after collision check (final output to odrive)
- `collision_monitor_state` (String) — Current collision zones status
- Costmaps: local (3m x 3m rolling) and global (full map)

## Action

- `/navigate_to_pose` (NavigateToPose) — Canonical goal dispatch interface

## Key Configuration (Tunable Parameters)

| Parameter | Value | File |
|-----------|-------|------|
| `desired_linear_vel` | 0.3 m/s | nav2_params.yaml |
| `lookahead_dist` | 0.4m (0.2-0.7m) | nav2_params.yaml |
| `goal_tolerance` | xy=0.15m, yaw=0.25 rad | nav2_params.yaml |
| `max_planning_time` | 2.0s | nav2_params.yaml |
| `controller_frequency` | 20 Hz | nav2_params.yaml |
| **Stop zone** | footprint + 5cm | collision_monitor.yaml |
| **Slowdown zone** | footprint + 25cm, 30% speed | collision_monitor.yaml |

**Robot footprint** (meters): `[[0.50, 0.37], [0.50, -0.37], [-0.30, -0.37], [-0.30, 0.37]]`

## Configuration Files

- `config/nav2_params.yaml` — Core Nav2 config (planner, controller, costmaps, behaviors)
- `config/collision_monitor.yaml` — Stop/slowdown zone polygons and sources
- `config/velocity_smoother.yaml` — cmd_vel smoothing parameters
- `config/slam_toolbox.yaml` — SLAM in mapping mode
- `config/slam_toolbox_localization.yaml` — SLAM in localization mode
- `config/nav2_hil_overrides.yaml` — HIL-specific parameter overrides
- `launch/navigation.launch.py` — Full Nav2 bringup with lifecycle management

## Collision Monitor Pipeline

```
cmd_vel -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel_safe
```

- Stop zone: immediate halt if obstacle inside footprint + 5cm
- Slowdown zone: 30% speed reduction if obstacle within footprint + 25cm
- Source: `/agv/scan` only (not depth, to avoid false positives)

## Dependencies

- nav2_bringup, nav2_bt_navigator, nav2_controller, nav2_planner, nav2_behaviors,
  nav2_collision_monitor, nav2_velocity_smoother, nav2_lifecycle_manager

## Improvement Opportunities

- Document which parameters are tunable for field commissioning vs fixed architectural choices
- Add field-calibration checklist for costmap and controller tuning
- Add Nav2 parameter validation (current config relies on Nav2 defaults for unset values)
- Consider adding costmap filter for keepout/speed zones (currently zones are UI-only)
