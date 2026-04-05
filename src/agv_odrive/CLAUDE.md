# agv_odrive

Production C++17 ROS2 driver for ODrive S1 motor controllers over SocketCAN.
Publishes differential-drive wheel odometry at 50 Hz and converts `cmd_vel` to motor velocity commands.

## Nodes

- **odrive_can_node** (C++17): Main production driver. Reads encoder feedback, computes odometry,
  sends velocity commands via CAN protocol.

## Dev Tools (Python, `dev_only: true`)

- `odrive_can_node.py` — CAN validation node (Python equivalent for commissioning)
- `odrive_gui.py` — ImGui diagnostic GUI for motor tuning
- `validate.py` — Hardware validation (CAN, OpenGL, ROS2, ODrive connectivity)
- `simple_test.py` — Basic CAN connectivity test

## Topics

**Published:**
- `wheel_odom` (nav_msgs/Odometry, 50 Hz) — Differential-drive odometry with dynamic covariance
- `joint_states` (sensor_msgs/JointState, 50 Hz) — Wheel angles and velocities (radians)
- `motor_state` (std_msgs/String, 2 Hz) — JSON: axis state, errors, voltage, temperatures
- `drive_debug` (std_msgs/String, 10 Hz) — JSON: command tracking, velocities, flags

**Subscribed:**
- `cmd_vel` (geometry_msgs/Twist) — Linear x + angular z velocity commands
- `e_stop` (std_msgs/Bool) — Emergency stop (true = immediate stop)
- `motor_enable` (std_msgs/Bool) — Arm/disarm motors (CLOSED_LOOP_CONTROL / IDLE)

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `can_interface` | `"can0"` | SocketCAN interface |
| `left_axis_id` | `0` | CAN node ID left motor |
| `right_axis_id` | `1` | CAN node ID right motor |
| `wheel_radius` | `0.0625` | Wheel radius (m) |
| `track_width` | `0.735` | Wheel-to-wheel distance (m) |
| `gear_ratio` | `1.0` | Motor turns / wheel turns |
| `publish_rate_hz` | `50` | Main loop frequency |
| `cmd_vel_timeout_ms` | `500` | Stop motors if no cmd_vel |
| `invert_left` | `false` | Negate left motor direction |
| `invert_right` | `true` | Negate right motor direction |
| `left_scale` / `right_scale` | `1.0` | Per-wheel velocity trim |
| `max_wheel_accel` | `1.0` | Rate limiter (turns/s^2) |
| `zero_vel_epsilon` | `0.03` | Velocity deadband (turns/s) |
| `min_effective_vel` | `0.0` | Stiction compensation minimum |
| `stiction_torque_ff` | `0.0` | Torque feedforward (Nm) |

## Key Algorithms

- **Odometry**: Mid-angle integration (more accurate than Euler for curved paths)
- **Inverse kinematics**: `v_left/right = (linear_x -/+ angular_z * track_width/2) / (wheel_radius * 2pi) * gear_ratio`
- **Wheel shaping pipeline**: Zero bypass -> acceleration limiter -> stiction compensation -> torque feedforward
- **CAN protocol**: Arbitration ID = `(node_id << 5) | cmd_id`, commands: HEARTBEAT, GET_ENCODER_ESTIMATES, SET_INPUT_VEL, SET_AXIS_STATE, GET_TEMPERATURE, GET_VBUS_VOLTAGE

## Configuration

- `config/odrive_params.yaml` — All parameters above
- `launch/odrive.launch.py` — Supports namespace, params_file, cmd_vel_topic remapping

## Dependencies

- SocketCAN (Linux kernel), rclcpp, nav_msgs, sensor_msgs, geometry_msgs, tf2

## Improvement Opportunities

- Add CAN retry with exponential backoff (currently logs spam if CAN unavailable)
- Add motor temperature shutdown thresholds (temps published but not monitored)
- Add integration test with mock CAN socket
- Add docstrings to Python dev tools
- Validate gear_ratio against ODrive firmware config to prevent silent misconfiguration
