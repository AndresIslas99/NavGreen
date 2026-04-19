# agv_description

Robot URDF/Xacro description for the AGV differential-drive platform. Defines chassis
geometry, wheel joints, and sensor mount positions. Publishes static TF tree via
robot_state_publisher.

## Nodes

- **robot_state_publisher** (from robot_state_publisher package): Parses URDF and
  publishes static TF transforms and joint states.

## URDF Structure

```
base_link (center of robot, 200mm above ground)
  +-- left_wheel (continuous joint, y=+0.3675m)
  +-- right_wheel (continuous joint, y=-0.3675m)
  +-- base_footprint (fixed, z=-0.200m, ground level)
```

**Note**: Camera TF (zed_camera_center) is NOT in the URDF. It is published by a
separate static_transform_publisher in agv_bringup/agv_slam launch files.

## Physical Parameters (from robot_params.yaml)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `wheel_radius` | `0.0625` m | 125mm diameter wheels |
| `track_width` | `0.735` m | Center-to-center wheel distance |
| `base_link_height` | `0.200` m | Base link above ground |
| `left_wheel_xyz` | `[0.0, 0.3675, -0.1375]` | Left wheel position |
| `right_wheel_xyz` | `[0.0, -0.3675, -0.1375]` | Right wheel position |
| `zed_camera_xyz` | `[0.700, 0.0, +0.010]` | Camera mount offset (210mm above ground; re-measured 2026-04-18) |

## URDF Files

- `urdf/agv_full.urdf.xacro` — Main composition (includes base + wheels)
- `urdf/agv_base.xacro` — Chassis link and base_footprint
- `urdf/wheel.xacro` — Wheel macro (continuous joints)

## Configuration

- `config/robot_params.yaml` — All physical dimensions (parameterized via Xacro)
- `launch/description.launch.py` — robot_state_publisher launch
- `launch/display.launch.py` — RViz visualization

## Dependencies

- robot_state_publisher, xacro, joint_state_publisher

## Improvement Opportunities

- Add URDF validation test (check_urdf / urdf_check)
- Add visual meshes for RViz visualization (currently primitive shapes)
- Document camera extrinsic calibration procedure for field deployment
- Add collision geometry for Gazebo simulation
