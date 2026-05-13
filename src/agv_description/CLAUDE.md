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

## Physical Parameters

Two YAMLs in `config/`:

- **`config/robot_geometry.yaml`** — runtime SSOT (loaded by
  `launch/description.launch.py` and passed to xacro as args; also
  loaded by `agv_odrive/launch/odrive.launch.py`). Single source of
  truth declared in
  `specs/persistence.yaml#config_artifacts.robot_geometry`. Sprint A of
  the 2026-05-13 audit (CRITICAL-02-02) currently parks runtime values
  here as `wheel_radius=0.0781`, `track_width=0.960`, `gear_ratio=10.0`
  — the bug-compensation factors carried until the ODrive NVRAM dump.
- **`config/robot_params.yaml`** — human-friendly documentary YAML with
  the geometric truth (radius 0.0625 m, track 0.735 m, base_link
  height, sensor mounts) and an ASCII-art layout diagram. NOT loaded
  at runtime; reference only. After CRITICAL-02-02 step 5 the values
  here will match `robot_geometry.yaml` exactly.

| Parameter | Geometric truth | Runtime (SSOT) | Purpose |
|-----------|-----------------|----------------|---------|
| `wheel_radius` | `0.0625` m | `0.0781` m (scaffold) | 125 mm diameter wheels (caliper-measured 2026-05-13) |
| `track_width` | `0.735` m | `0.960` m (scaffold) | Center-to-center wheel distance |
| `base_link_height` | `0.200` m | (same) | Base link above ground |
| `left_wheel_xyz` | `[0.0, 0.3675, -0.1375]` | `[0.0, 0.480, -0.1375]` (scaffold) | Left wheel position (y derived from track_width / 2) |
| `right_wheel_xyz` | `[0.0, -0.3675, -0.1375]` | `[0.0, -0.480, -0.1375]` (scaffold) | Right wheel position |
| `zed_camera_xyz` | `[0.700, 0.0, +0.010]` | (same — not in SSOT yet) | Camera mount offset (210 mm above ground; re-measured 2026-04-18). Pending HIGH-01-02 in audit. |

`verify_geometry_ssot.py` enforces SSOT structurally and emits `WARN`
lines on the numerical divergence above; both WARN lines clear when
CRITICAL-02-02 step 5 lands.

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
