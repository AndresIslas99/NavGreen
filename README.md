# AGV Greenhouse

Autonomous differential-drive robot for commercial greenhouse deployment in Mexico.
ROS2 (Humble) workspace targeting Jetson AGX Orin 64GB (development) and Jetson Orin NX 16GB (production).

**MVP goal**: First field visit with local WiFi operator workflow — teleoperation, mapping,
waypoint missions, and live monitoring from a browser tablet.

## Architecture

```
 Dashboard (React)  <--WebSocket/REST-->  agv_ui_backend (TypeScript/rclnodejs)
                                               |
                         +---------------------+---------------------+
                         |                     |                     |
                   agv_navigation        agv_map_manager      agv_waypoint_manager
                   (Nav2 stack)          (map persistence)    (mission CRUD)
                         |
                   agv_sensor_fusion (dual EKF)
                    /          \
              ekf_local      ekf_global
              (50 Hz)        (10 Hz)
               /    \          /    \
         agv_odrive  IMU   cuVSLAM  agv_markers
         (CAN motor)       (visual  (AprilTag
          driver)           SLAM)   correction)
```

See `docs/architecture.md` for detailed system diagrams.

## Packages

| Package | Language | Purpose | Status |
|---------|----------|---------|--------|
| **agv_odrive** | C++17 + Python dev | ODrive S1 CAN driver, wheel odometry @ 50Hz | in_progress |
| **agv_description** | Xacro/URDF | Robot geometry, TF tree, sensor mounts | built |
| **agv_bringup** | Python launch | Launch orchestration for all operational modes | built |
| **agv_sensor_fusion** | C++17 config | Dual EKF: local (odom->base_link) + global (map->odom) | hil_validated |
| **agv_navigation** | Nav2 config | Path planning, trajectory following, collision monitor | hil_validated |
| **agv_behaviors** | C++17 + BT XML | Behavior tree mission execution | built (MVP) |
| **agv_map_manager** | C++17 | Map save/load, keepout/speed zone persistence | built |
| **agv_waypoint_manager** | C++17 | Mission CRUD, sequential waypoint dispatch | built |
| **agv_markers** | C++17 | AprilTag pose correction (post-MVP) | built |
| **agv_scan_mapper** | C++17 | Live 2D occupancy grid from LaserScan | built |
| **agv_image_server** | C++17 | MJPEG HTTP streaming for camera/depth | built |
| **agv_interfaces** | ROS2 IDL | Custom messages (2) and services (6) | built |
| **agv_ui_backend** | TypeScript | WebSocket/REST bridge for dashboard | built |
| **agv_integration_tests** | Python | System-level integration tests | built |

**Web frontend**: `web/agv_dashboard/` (React/TypeScript, ISA-101 industrial design)

## Build

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# First-time clone: pull external dependencies pinned in agv_greenhouse.repos.
# This file lists Isaac ROS, ZED wrapper, Nav2, SLAM Toolbox, apriltag_ros and
# the other packages the production launch files include from outside src/.
# See CR-00-01 in docs/audit/2026-05-13-greenhouse-hardening/00_inventory.md
# for the motivation. Skip if your workspace already has them installed.
vcs import src < agv_greenhouse.repos

# Build all packages (warnings as errors)
colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-Werror"

# Run tests
colcon test
colcon test-result --verbose
```

## Launch Modes

| Mode | Command | Use Case |
|------|---------|----------|
| Full stack | `ros2 launch agv_bringup agv_full.launch.py map:=<path>` | Production autonomy |
| Robot core | `ros2 launch agv_bringup agv_robot_core.launch.py` | Motor control + odom only |
| Teleop | `ros2 launch agv_bringup agv_teleop.launch.py` | Remote operation |
| Mapping | `ros2 launch agv_bringup agv_mapping.launch.py` | Map commissioning |
| HIL | `ros2 launch agv_bringup agv_hil_full.launch.py` | Simulation testing |

## Hardware

- **Compute**: Jetson AGX Orin 64GB (dev) / Orin NX 16GB (prod)
- **Motors**: 2x M8325s BLDC via ODrive S1 (CAN bus, 250 kbps)
- **Camera**: ZED 2i stereo (RGB + depth + IMU)
- **Kinematics**: Differential drive, 125mm wheel diameter, 735mm track width

See `docs/hardware_setup.md` for CAN bus pinmux and ODrive configuration.

## Canonical Sources

1. `specs/project.yaml` — Project scope, phases, success criteria
2. `specs/interfaces.yaml` — ROS2 topics, services, actions, TF tree
3. `specs/acceptance.yaml` — Quality gates per phase
4. `agents/registry.yaml` — Agent roles and coordination
5. `policies/engineering_rules.md` — Development rules and constraints

## Operational Documentation

- `docs/architecture.md` — System architecture diagrams
- `docs/hardware_setup.md` — Jetson CAN pinmux, ODrive setup
- `docs/mapping_commissioning.md` — Map creation procedure
- `docs/dual_ekf_validation.md` — Localization validation steps
- `docs/low_speed_validation.md` — Drivetrain commissioning checklist
- `docs/production_readiness_assessment.md` — Production readiness review

## Key Constraints

- All ROS2 robot nodes are **C++17 only** (no Python in runtime stack)
- No hardcoded physical parameters, IPs, or marker IDs
- Build warnings treated as errors
- All configuration from YAML or environment variables
- See `CLAUDE.md` for full development rules
