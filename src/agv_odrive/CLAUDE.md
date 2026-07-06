# agv_odrive

Production C++17 ROS2 driver for ODrive S1 motor controllers over SocketCAN.
Publishes differential-drive wheel odometry at 50 Hz and converts `cmd_vel` to motor velocity commands.

## Nodes

- **odrive_can_node** (C++17): Main production driver. Reads encoder feedback, computes odometry,
  sends velocity commands via CAN protocol.

## Dev Tools (Python, in `scripts/` — commissioning only, never in the robot runtime)

- `odrive_can_node.py` — CAN validation node (Python equivalent for commissioning)
- `odrive_gui.py` — ImGui diagnostic GUI for motor tuning
- `validate.py` — Hardware validation (CAN, OpenGL, ROS2, ODrive connectivity)
- `simple_test.py` — Basic CAN connectivity test
- `calibrate_odom.py` — Two-step wheel_radius / track_width calibration
- `check_odrive_config.py` — ODrive S1 tuning config checker (USB)
- `auto_calibrate.py` — Automated ODrive calibration helper

## Topics

**Published:**
- `wheel_odom` (nav_msgs/Odometry, 50 Hz) — Differential-drive odometry with dynamic covariance
- `joint_states` (sensor_msgs/JointState, 50 Hz) — Wheel angles and velocities (radians)
- `motor_state` (std_msgs/String, 10 Hz) — JSON: axis state, errors, voltage, temperatures, `feedback_ok`
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
| `wheel_radius` | *(required, YAML)* | Wheel radius (m) — 0.0781 calibrated 2026-04-08; no in-code default |
| `track_width` | *(required, YAML)* | Effective wheel-to-wheel distance (m) — 0.960 calibrated; no in-code default |
| `gear_ratio` | *(required, YAML)* | Motor turns / wheel turns — 10.0 for the 10:1 planetary; no in-code default |
| `publish_rate_hz` | `50` | Main loop frequency |
| `cmd_vel_timeout_ms` | `200` | Stop motors if no cmd_vel (was 500) |
| `feedback_timeout_ms` | `300` | Feedback watchdog: fault if no heartbeat/encoder frame for this long (0 disables) |
| `invert_left` | `true` | Negate left motor direction |
| `invert_right` | `false` | Negate right motor direction |
| `left_scale` / `right_scale` | `1.0` | Per-wheel velocity trim |
| `max_wheel_accel` | `0.5` | Acceleration rate limiter (turns/s²) |
| `max_wheel_decel` | `1.5` | Deceleration rate limiter (turns/s²) — 3x faster than accel |
| `zero_vel_epsilon` | `0.03` | Velocity deadband (turns/s) |
| `min_effective_vel` | `0.0` | Stiction compensation minimum |
| `stiction_torque_ff` | `0.03` | Torque feedforward (Nm) — compensates caster friction |
| `velocity_filter_alpha` | `0.3` | EMA low-pass coefficient (0=smooth, 1=raw). ~8Hz cutoff at 50Hz |
| `slip_velocity_threshold` | `0.5` | Encoder diff to trigger slip detection (turns/s) |
| `slip_reduction_factor` | `0.7` | Scale cmd_vel by this during slip |
| `slip_cooldown_ms` | `200.0` | Hold slip reduction after clearance (ms) |
| `caster_enable_compensation` | `true` | Enable caster disturbance covariance inflation |
| `caster_settling_tau` | `0.5` | Caster disturbance decay time constant (s) |
| `caster_covariance_multiplier` | `10.0` | Peak covariance inflation factor |
| `caster_angular_accel_threshold` | `1.0` | Angular accel trigger for caster (rad/s²) |

## Key Algorithms

- **Odometry**: Mid-angle integration (more accurate than Euler for curved paths)
- **EMA velocity filter**: Low-pass on encoder velocities (α=0.3) to smooth noise before twist and rotation detection
- **Pure rotation gate**: When wheels spin opposite with similar magnitude, forces symmetric deltas to eliminate phantom translation (delta_s=0 exactly)
- **Wheel slip detection**: If encoder velocity difference exceeds threshold during straight-line command, reduces cmd_vel to 70% and inflates odometry covariance
- **Asymmetric accel limiter**: Deceleration (1.5 turns/s²) is 3x faster than acceleration (0.5 turns/s²) for responsive stopping; uses the measured inter-command interval (clamped 1–100 ms) so the limit holds at any upstream publish rate
- **Feedback watchdog**: If no heartbeat/encoder frame arrives within `feedback_timeout_ms`, the node reports motors disarmed, zeroes the published twist, inflates odometry covariance 100x, and re-arms the odometry first-sample gate for a clean re-latch when feedback returns
- **Caster compensation**: Inflates odometry covariance during direction changes AND sustained rotation (>0.3 rad/s), signaling EKF to trust IMU more
- **Inverse kinematics**: `v_left/right = (linear_x -/+ angular_z * track_width/2) / (wheel_radius * 2pi) * gear_ratio`
- **Wheel shaping pipeline**: Zero bypass -> asymmetric accel/decel limiter -> stiction compensation -> torque feedforward
- **CAN protocol**: Arbitration ID = `(node_id << 5) | cmd_id`, commands: HEARTBEAT, GET_ENCODER_ESTIMATES, SET_INPUT_VEL, SET_AXIS_STATE, GET_TEMPERATURE, GET_VBUS_VOLTAGE

## Configuration

- `config/odrive_params.yaml` — All parameters above
- `launch/odrive.launch.py` — Supports namespace, params_file, cmd_vel_topic remapping

## Dependencies

- SocketCAN (Linux kernel), rclcpp, nav_msgs, sensor_msgs, geometry_msgs, tf2

## Improvement Opportunities

- Enable and validate the ODrive firmware axis watchdog (`axis.config.enable_watchdog`)
  so a host crash disarms the motors controller-side (needs hardware validation)
- Add integration test with mock CAN socket (RTR-frame regression test)
- Add docstrings to Python dev tools
- Validate gear_ratio against ODrive firmware config to prevent silent misconfiguration
