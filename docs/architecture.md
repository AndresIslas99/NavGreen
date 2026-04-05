# AGV Greenhouse — System Architecture

## High-Level Overview

```
                          +--------------------+
                          |   agv_dashboard    |
                          |  (React/TypeScript)|
                          +--------+-----------+
                                   |  WebSocket / REST
                          +--------+-----------+
                          |  agv_ui_backend    |
                          |  (TypeScript/      |
                          |   rclnodejs :8090) |
                          +--------+-----------+
                                   |  ROS2 topics / actions / services
            +----------------------+----------------------+
            |                      |                      |
  +---------v--------+  +---------v--------+  +-----------v----------+
  |  agv_navigation  |  | agv_map_manager  |  | agv_waypoint_manager |
  |  (Nav2 stack)    |  | (map persistence)|  | (mission CRUD)       |
  +--------+---------+  +------------------+  +----------------------+
           |
           | /navigate_to_pose
  +--------v---------+
  |  agv_behaviors   |
  | (BehaviorTree)   |
  +------------------+
           |
  +--------v---------+     +-------------------+
  | agv_sensor_fusion|<----| agv_markers       |
  | (dual EKF)       |     | (AprilTag drift   |
  +---+---------+----+     |  correction)      |
      |         |           +-------------------+
      |         |
+-----v---+ +--v-----------+     +-------------------+
|ekf_local | |ekf_global    |     | agv_scan_mapper   |
|50Hz      | |10Hz          |     | (occupancy grid)  |
|odom->    | |map->odom     |     +-------------------+
|base_link | |              |             ^
+----+-----+ +---+-----+---+             |
     |            |     |          /agv/scan
     |            |     |                |
+----v-----+  +--v--+ +v-----------+  +-+------------------+
|agv_odrive|  |IMU  | |cuVSLAM     |  |pointcloud_to_laser |
|(CAN motor|  |(ZED)| |(visual     |  |(height filter)     |
| driver)  |  +-----+ | SLAM)      |  +--------------------+
+----------+           +------------+          ^
     |                      |                  |
     v                      v                  |
  ODrive S1            ZED 2i stereo camera ---+
  (SocketCAN)          (stereo + IMU + depth)
```

## Localization Pipeline (Dual EKF)

```
 wheel_odom (50 Hz)  ----+
   from agv_odrive        |
                          v
                    +-----------+
 ZED IMU (200 Hz) ->| ekf_local |---> odometry/local (50 Hz)
                    | (odom     |---> TF: odom -> base_link
                    |  frame)   |          |
                    +-----------+          |
                                           v
                                    +------------+
 cuVSLAM (10-15 Hz) -------------->| ekf_global |---> odometry/global (10 Hz)
   /visual_slam/tracking/odometry  | (map frame)|---> TF: map -> odom
                                   |            |
 marker_pose (event) ------------->|            |---> /agv/pose (10 Hz)
   from agv_markers                +------------+     via fusion_monitor
```

**Key design**: Local EKF provides continuous smooth odometry. Global EKF corrects drift
using visual SLAM and optional AprilTag markers. If cuVSLAM degrades (lighting, repetitive
textures), local estimate remains usable.

## Perception Pipeline

```
 ZED 2i stereo camera
    |
    +---> cuVSLAM (stereo-inertial SLAM)
    |       --> /visual_slam/tracking/odometry (to global EKF)
    |
    +---> pointcloud_to_laserscan (height filter 0.05-1.20m)
    |       --> /agv/scan
    |             |
    |             +---> scan_grid_mapper --> /agv/live_map (commissioning)
    |             +---> Nav2 costmap layers (navigation)
    |
    +---> agv_image_server (MJPEG HTTP :8091)
    |       --> /camera/stream, /depth/stream
    |
    +---> isaac_ros_apriltag (optional, post-MVP)
            --> /agv/tag_detections
                  |
                  +---> marker_correction_node
                          --> /agv/marker_pose (to global EKF)
                          --> relocalization (set_pose if drift > 2m)
```

## Navigation Stack

```
 /navigate_to_pose action
    |
    v
 +--bt_navigator-----------------------------------------+
 |  behavior tree (single_waypoint.xml or recovery tree) |
 +---+---------------------------------------------------+
     |
     +---> planner_server (SmacPlanner2D / Hybrid-A*)
     |       - global costmap (static + obstacle + inflation)
     |       - max planning time: 2.0s
     |
     +---> controller_server (RegulatedPurePursuit)
     |       - local costmap (3m x 3m rolling, 0.05m resolution)
     |       - desired velocity: 0.3 m/s
     |       - goal tolerance: xy=0.15m, yaw=0.25 rad
     |
     +---> behavior_server (spin, backup, wait)
     |       - recovery actions for stuck conditions
     |
     v
 cmd_vel --> velocity_smoother --> cmd_vel_smoothed
                                       |
                                       v
                                collision_monitor
                                  - stop zone (footprint + 5cm)
                                  - slowdown zone (footprint + 25cm, 30% speed)
                                       |
                                       v
                                 cmd_vel_safe --> agv_odrive
```

## TF Frame Tree

```
map                          (owned by: ekf_global)
 +-- odom                    (owned by: ekf_local)
      +-- base_link          (robot center, 200mm above ground)
           +-- left_wheel    (continuous joint, robot_state_publisher)
           +-- right_wheel   (continuous joint, robot_state_publisher)
           +-- base_footprint (fixed, z=-0.200, ground level)

 zed_camera_center           (static TF from agv_slam launch)
   +-- zed_left_camera_frame
   +-- zed_right_camera_frame
   +-- imu_link
```

**Critical**: cuVSLAM must NOT publish TF (`publish_odom_to_base_tf: false`,
`publish_map_to_odom_tf: false`). EKF nodes own the TF tree exclusively.

## UI Architecture

```
 +---agv_dashboard (React :5173)--------+
 |  Map viewer + Teleop + Missions      |
 |  ISA-101 industrial design           |
 +----------+---------------------------+
            | WebSocket (real-time)
            | REST API (CRUD)
 +----------v---------------------------+
 |  agv_ui_backend (Express :8090)      |
 |  State machine (derive_state)        |
 |  Event log (JSONL persistence)       |
 |  Telemetry store (time-series)       |
 +----------+---------------------------+
            | ROS2 (rclnodejs)
            |
  +---------+----------+----------+
  |         |          |          |
 cmd_vel  e_stop  navigate_  map_manager/
                  to_pose   waypoint_manager
                  (action)  (services)
```

**State machine states**: offline, idle, ready, mapping, navigating,
executing_mission, blocked, e_stop, fault.

## Startup Sequence (agv_full.launch.py)

| Delay | Component | Purpose |
|-------|-----------|---------|
| 0s | robot_state_publisher | Static TF (URDF) |
| 0s | odrive_can_node | Motor control + wheel odom @ 50Hz |
| 0s | pointcloud_to_laserscan | Ground-filtered scan |
| 0s | teleop_server | Dashboard backend :8090 |
| 2s | cuVSLAM | Visual SLAM (TF disabled) |
| 4s | ekf_local + ekf_global | Dual EKF sensor fusion |
| 5s | map_manager + waypoint_manager | Map/mission persistence |
| 6s | Nav2 stack | Autonomous navigation |
| 7s | AprilTag detection (optional) | Marker correction |
| 7s | Behavior executor (optional) | Mission BT execution |

## Package Dependency Graph

```
agv_interfaces (msg/srv definitions)
    ^
    |--- agv_map_manager
    |--- agv_waypoint_manager
    |--- agv_behaviors
    |--- agv_ui_backend

agv_description (URDF/TF)
    ^
    |--- agv_bringup (launch orchestration)
            ^
            |--- agv_odrive
            |--- agv_sensor_fusion (robot_localization)
            |--- agv_navigation (Nav2)
            |--- agv_markers (AprilTag)
            |--- agv_scan_mapper
            |--- agv_image_server

agv_integration_tests (validates all above)
```

## Hardware Topology

```
Jetson AGX Orin 64GB (development)
    |
    +-- USB: ZED 2i stereo camera
    |         (RGB + depth + IMU)
    |
    +-- CAN0 (SocketCAN, 250 kbps):
    |     +-- ODrive S1 axis 0 (left motor, M8325s)
    |     +-- ODrive S1 axis 1 (right motor, M8325s)
    |
    +-- WiFi: local network
              (tablet operator connects via browser)
```

CAN bus requires Jetson pinmux configuration. See `docs/hardware_setup.md`.
