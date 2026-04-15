# agv_hw_interface

ros2_control `SystemInterface` plugin for the AGV ODrive CAN driver. **Co-exists**
with [src/agv_odrive/](../agv_odrive/) — this package is opt-in via its own
launch files; the legacy standalone driver remains the default production motor
node until this flow is field-validated.

This is Gap 1 of [docs/architectural_gaps.md](../../docs/architectural_gaps.md).

## Build prerequisite (one-time)

The package depends on the ros2_control stack, which is not installed by
default on a fresh Jetson. Until installed, a `COLCON_IGNORE` file at the
root of this package keeps colcon from attempting to build it. To unblock:

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-hardware-interface
rm src/agv_hw_interface/COLCON_IGNORE
colcon build --packages-select agv_hw_interface
```

## What this package provides

1. **Plugin** `agv_hw_interface/AgvDiffDriveSystem` — wraps the ODrive CAN
   protocol behind ros2_control's `SystemInterface`. Exposes
   `left_wheel_joint` and `right_wheel_joint` with `position` + `velocity`
   state interfaces and a `velocity` command interface each.
2. **URDF variant** [urdf/agv_ros2control.urdf.xacro](urdf/agv_ros2control.urdf.xacro)
   that includes the existing geometry from [agv_description](../agv_description/)
   and appends a `<ros2_control>` block.
3. **Controllers config** [config/agv_controllers.yaml](config/agv_controllers.yaml)
   with `controller_manager` + `joint_state_broadcaster` + `diff_drive_controller`.
4. **Two launch files**:
   - [launch/agv_ros2control.launch.py](launch/agv_ros2control.launch.py) — real CAN
   - [launch/agv_ros2control_mock.launch.py](launch/agv_ros2control_mock.launch.py) — mock_components/GenericSystem (no hardware)

## Why this exists (the developer-velocity argument)

Without ros2_control + mock_components, anyone working on navigation,
behaviors, or the dashboard needs the physical robot or has to mock topics by
hand. With this package, mock hardware is one launch command away:

```
ros2 launch agv_hw_interface agv_ros2control_mock.launch.py
ros2 control list_controllers
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: ""}, twist: {linear: {x: 0.2}}}'
```

## Why CAN code is duplicated from agv_odrive

[include/agv_hw_interface/can_socket.hpp](include/agv_hw_interface/can_socket.hpp)
and [include/agv_hw_interface/odrive_protocol.hpp](include/agv_hw_interface/odrive_protocol.hpp)
are duplicated from agv_odrive. The duplication is intentional and limited:

- agv_odrive does not export a library — it builds an executable. Linking
  against its symbols would require restructuring its CMakeLists, which would
  destabilize the production driver.
- The duplicated code is small (CAN socket + ~50 lines of protocol packing).
- When the migration in
  [docs/architectural_gaps.md](../../docs/architectural_gaps.md) Gap 1 is
  fully complete and agv_odrive is removed, this becomes the only copy.

## TF ownership

`diff_drive_controller` is configured with `enable_odom_tf: false` so it
publishes `/odom` (Odometry) but **does not** publish the `odom -> base_link`
transform. The dual-EKF stack in [agv_sensor_fusion](../agv_sensor_fusion/)
remains the sole owner of `odom -> base_link` and `map -> odom`. This matches
the existing TF ownership documented in
[src/agv_bringup/CLAUDE.md](../agv_bringup/CLAUDE.md).

## Topics produced

- `/diff_drive_controller/odom` — Odometry (replaces `wheel_odom` from agv_odrive)
- `/joint_states` — JointState (replaces `joint_states` from agv_odrive)
- `/diff_drive_controller/cmd_vel` — TwistStamped subscription

## Topics NOT produced (still need agv_odrive for these)

- `/agv/motor_state` — JSON axis state, errors, voltage, temperatures
- `/agv/drive_debug` — JSON command tracking
- `/agv/e_stop` subscription — software E-stop input
- caster compensation, wheel slip detection, asymmetric accel limiter,
  velocity filter, mid-angle integration

These features live in the original
[agv_odrive standalone node](../agv_odrive/) and are not replicated here. The
plugin's `read()`/`write()` methods are intentionally minimal — the goal of
this package is to unblock parallel development with mock hardware, not to
replace agv_odrive's tuned behavior on real hardware.

When the user is ready to fully migrate, the next steps are documented in
[TASK.yaml#follow_up_work](TASK.yaml).

## Testing

```
colcon test --packages-select agv_hw_interface --event-handlers console_direct+
```

`test_kinematics` exercises the radians<->turns conversion math without a
SystemInterface instance, so it runs in any environment.

## Improvement opportunities

- Move the duplicated CAN code into a shared `agv_can` library and depend on
  it from both packages
- Add a launch_testing test that brings up `agv_ros2control_mock.launch.py`
  and asserts the controller transitions to `active`
- Port the caster compensation, slip detection, and accel shaping from
  agv_odrive into the plugin so it becomes a full replacement
- Lifecycle node variant of controller_manager
