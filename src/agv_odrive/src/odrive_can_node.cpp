#include "agv_odrive/odrive_can_node.hpp"

#include <cmath>
#include <linux/can.h>

namespace agv_odrive {

ODriveCANNode::ODriveCANNode() : Node("agv_odrive_node") {
  // -- Declare parameters --
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("left_axis_id", 0);
  this->declare_parameter("right_axis_id", 1);
  this->declare_parameter("wheel_radius", 0.0625);
  this->declare_parameter("track_width", 0.735);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("publish_rate_hz", 50);
  this->declare_parameter("cmd_vel_timeout_ms", 500);
  this->declare_parameter("invert_left", true);
  this->declare_parameter("invert_right", false);
  this->declare_parameter("left_scale", 1.0);
  this->declare_parameter("right_scale", 1.0);
  this->declare_parameter("min_effective_vel", 0.0);
  this->declare_parameter("stiction_torque_ff", 0.0);
  this->declare_parameter("max_wheel_accel", 1.0);
  this->declare_parameter("max_wheel_decel", 1.5);
  this->declare_parameter("zero_vel_epsilon", 0.03);
  // gear_ratio = motor_turns / wheel_turns. Set to 1.0 if ODrive firmware already
  // has gear_ratio configured. Set to 10.0 for a 10:1 planetary gearbox with raw
  // encoder feedback.
  this->declare_parameter("gear_ratio", 1.0);
  this->declare_parameter("max_fet_temp", 70.0);
  this->declare_parameter("max_motor_temp", 80.0);
  this->declare_parameter("critical_temp_offset", 10.0);
  this->declare_parameter("caster_enable_compensation", true);
  this->declare_parameter("caster_settling_tau", 0.5);
  this->declare_parameter("caster_covariance_multiplier", 10.0);
  this->declare_parameter("caster_angular_accel_threshold", 1.0);
  this->declare_parameter("velocity_filter_alpha", 0.3);
  this->declare_parameter("slip_velocity_threshold", 0.5);
  this->declare_parameter("slip_reduction_factor", 0.7);
  this->declare_parameter("slip_cooldown_ms", 200.0);

  // -- Read parameters --
  can_interface_ = this->get_parameter("can_interface").as_string();
  left_axis_id_ = static_cast<uint8_t>(this->get_parameter("left_axis_id").as_int());
  right_axis_id_ = static_cast<uint8_t>(this->get_parameter("right_axis_id").as_int());
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  track_width_ = this->get_parameter("track_width").as_double();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
  cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();
  invert_left_ = this->get_parameter("invert_left").as_bool();
  invert_right_ = this->get_parameter("invert_right").as_bool();
  left_sign_ = invert_left_ ? -1.0 : 1.0;
  right_sign_ = invert_right_ ? -1.0 : 1.0;
  left_scale_ = this->get_parameter("left_scale").as_double();
  right_scale_ = this->get_parameter("right_scale").as_double();
  min_effective_vel_ = static_cast<float>(this->get_parameter("min_effective_vel").as_double());
  stiction_torque_ff_ = static_cast<float>(this->get_parameter("stiction_torque_ff").as_double());
  max_wheel_accel_ = static_cast<float>(this->get_parameter("max_wheel_accel").as_double());
  max_wheel_decel_ = static_cast<float>(this->get_parameter("max_wheel_decel").as_double());
  zero_vel_epsilon_ = static_cast<float>(this->get_parameter("zero_vel_epsilon").as_double());
  gear_ratio_ = this->get_parameter("gear_ratio").as_double();
  max_fet_temp_ = this->get_parameter("max_fet_temp").as_double();
  max_motor_temp_ = this->get_parameter("max_motor_temp").as_double();
  critical_temp_offset_ = this->get_parameter("critical_temp_offset").as_double();
  caster_enable_compensation_ = this->get_parameter("caster_enable_compensation").as_bool();
  caster_settling_tau_ = this->get_parameter("caster_settling_tau").as_double();
  caster_covariance_multiplier_ = this->get_parameter("caster_covariance_multiplier").as_double();
  caster_angular_accel_threshold_ = this->get_parameter("caster_angular_accel_threshold").as_double();
  velocity_filter_alpha_ = std::clamp(this->get_parameter("velocity_filter_alpha").as_double(), 0.01, 1.0);
  slip_velocity_threshold_ = this->get_parameter("slip_velocity_threshold").as_double();
  slip_reduction_factor_ = this->get_parameter("slip_reduction_factor").as_double();
  slip_cooldown_ms_ = this->get_parameter("slip_cooldown_ms").as_double();

  // -- Validate --
  if (wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "wheel_radius must be > 0, got %f", wheel_radius_);
    throw std::runtime_error("Invalid wheel_radius");
  }
  if (track_width_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "track_width must be > 0, got %f", track_width_);
    throw std::runtime_error("Invalid track_width");
  }
  if (gear_ratio_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "gear_ratio must be > 0, got %f", gear_ratio_);
    throw std::runtime_error("Invalid gear_ratio");
  }
  if (caster_settling_tau_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "caster_settling_tau must be > 0, got %f", caster_settling_tau_);
    throw std::runtime_error("Invalid caster_settling_tau");
  }
  if (caster_covariance_multiplier_ < 1.0) {
    RCLCPP_FATAL(get_logger(), "caster_covariance_multiplier must be >= 1.0, got %f",
                 caster_covariance_multiplier_);
    throw std::runtime_error("Invalid caster_covariance_multiplier");
  }

  // -- NVRAM-audit diagnostic (CRITICAL-02-02 / MEDIUM-02-07) --
  // The SSOT at src/agv_description/config/robot_geometry.yaml currently
  // carries field-validated compensation values (wheel_radius 0.0781,
  // track_width 0.960, gear_ratio 10.0) rather than the geometric truth
  // (caliper measurement confirms wheel_radius = 0.0625 m). The 1.25×
  // factor is suspected to live in ODrive S1 NVRAM — typically
  // encoder.config.cpr or motor.config.pole_pairs. Log the live
  // kinematics at every boot so the operator can cross-check against
  // `odrivetool` (procedure: docs/calibration/odrive_nvram_dump_procedure.md).
  // If ROS gear_ratio != 1.0 AND ODrive NVRAM also has a non-unity
  // gear_ratio, motors are double-counted — verify before deployment.
  RCLCPP_INFO(get_logger(),
              "Kinematics SSOT: wheel_radius=%.4fm, track_width=%.4fm, gear_ratio=%.2f. "
              "Verify against ODrive NVRAM (encoder.cpr, motor.config.pole_pairs, "
              "motor.config.gear_ratio) per docs/calibration/odrive_nvram_dump_procedure.md.",
              wheel_radius_, track_width_, gear_ratio_);

  // -- Publishers --
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
  pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_motor_state_ = this->create_publisher<std_msgs::msg::String>("motor_state", 10);
  pub_drive_debug_ = this->create_publisher<std_msgs::msg::String>("drive_debug", 10);

  // -- Subscribers --
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&ODriveCANNode::on_cmd_vel, this, std::placeholders::_1));
  sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "e_stop", 10, std::bind(&ODriveCANNode::on_e_stop, this, std::placeholders::_1));
  sub_motor_enable_ = this->create_subscription<std_msgs::msg::Bool>(
    "motor_enable", 10, std::bind(&ODriveCANNode::on_motor_enable, this, std::placeholders::_1));

  // -- Initialize CAN --
  if (!init_can()) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN interface %s. Will retry in timers.",
                 can_interface_.c_str());
  }

  last_odom_time_ = this->now();
  last_cmd_vel_time_ = this->now();

  // -- Timers --
  auto main_period = std::chrono::milliseconds(1000 / publish_rate_hz_);
  timer_main_ = this->create_wall_timer(main_period,
    std::bind(&ODriveCANNode::main_loop, this));

  // Encoder polling must match main loop rate to avoid stale-sample staircase artifacts
  timer_encoder_ = this->create_wall_timer(std::chrono::milliseconds(20),
    std::bind(&ODriveCANNode::encoder_request_loop, this));

  // Motor state at 10 Hz (100ms) — higher frequency ensures reliable DDS
  // discovery between C++ and rclnodejs subscribers on same machine.
  timer_motor_state_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&ODriveCANNode::publish_motor_state, this));

  timer_debug_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&ODriveCANNode::publish_drive_debug, this));

  RCLCPP_INFO(get_logger(), "ODrive CAN node started on %s (left=%d, right=%d)",
              can_interface_.c_str(), left_axis_id_, right_axis_id_);
  RCLCPP_INFO(get_logger(), "wheel_radius=%.4f m, track_width=%.4f m, rate=%d Hz, gear_ratio=%.2f",
              wheel_radius_, track_width_, publish_rate_hz_, gear_ratio_);
  RCLCPP_INFO(get_logger(), "invert_left=%s, invert_right=%s",
              invert_left_ ? "true" : "false", invert_right_ ? "true" : "false");
  if (caster_enable_compensation_) {
    RCLCPP_INFO(get_logger(), "Caster compensation: tau=%.2fs, multiplier=%.1f, accel_thresh=%.2f rad/s²",
                caster_settling_tau_, caster_covariance_multiplier_, caster_angular_accel_threshold_);
  }
}

ODriveCANNode::~ODriveCANNode() {
  stop_motors();
}

// ── CAN initialization ──

bool ODriveCANNode::init_can() {
  can_ = std::make_unique<CANSocket>(can_interface_);
  if (!can_->is_open()) {
    can_.reset();
    return false;
  }
  can_->set_filter(left_axis_id_, right_axis_id_);
  RCLCPP_INFO(get_logger(), "CAN socket opened on %s", can_interface_.c_str());
  return true;
}

void ODriveCANNode::check_temperature(const AxisData& axis) {
  double max_temp = std::max(
    static_cast<double>(axis.fet_temp),
    static_cast<double>(axis.motor_temp));

  if (max_temp > max_fet_temp_ + critical_temp_offset_ ||
      axis.motor_temp > max_motor_temp_ + critical_temp_offset_) {
    thermal_state_ = "critical";
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "CRITICAL: Temperature limit exceeded (FET=%.1f Motor=%.1f), disabling motors",
      axis.fet_temp, axis.motor_temp);
    stop_motors();
  } else if (axis.fet_temp > max_fet_temp_ || axis.motor_temp > max_motor_temp_) {
    thermal_state_ = "warning";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Temperature warning: FET=%.1f (limit %.1f) Motor=%.1f (limit %.1f)",
      axis.fet_temp, max_fet_temp_, axis.motor_temp, max_motor_temp_);
  } else {
    thermal_state_ = "ok";
  }
}

// ── Main loop (50 Hz) ──

void ODriveCANNode::main_loop() {
  // Retry CAN with exponential backoff
  if (!can_ || !can_->is_open()) {
    auto now = this->now();
    auto elapsed = (now - last_can_retry_).nanoseconds() / 1000000;
    if (elapsed < can_retry_delay_ms_) return;
    last_can_retry_ = now;
    if (!init_can()) {
      can_retry_delay_ms_ = std::min(can_retry_delay_ms_ * 2, 3000);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "CAN init failed on %s, next retry in %dms",
        can_interface_.c_str(), can_retry_delay_ms_);
      return;
    }
    RCLCPP_INFO(get_logger(), "CAN connection restored on %s", can_interface_.c_str());
    can_retry_delay_ms_ = 100;
  }

  // Read all pending CAN messages
  read_can_messages();

  // cmd_vel timeout: stop if no command received recently
  auto elapsed_ms = (this->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
  if (elapsed_ms > cmd_vel_timeout_ms_ && !e_stop_active_) {
    left_.prev_cmd  = 0.0f;
    right_.prev_cmd = 0.0f;
    last_left_target_  = 0.0f;
    last_right_target_ = 0.0f;
    zero_cmd_active_ = true;
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
  }

  // Integrate and publish odometry
  integrate_odometry();
  publish_odometry();
  publish_joint_states();
}

// ── Encoder request (50 Hz) ──

void ODriveCANNode::encoder_request_loop() {
  if (!can_ || !can_->is_open()) return;

  can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_ENCODER_ESTIMATES));
  can_->send_rtr(make_arb_id(right_axis_id_, cmd::GET_ENCODER_ESTIMATES));

  // Request temperature and voltage at 1Hz (every 50th call at 50Hz)
  if (++diag_counter_ % 50 == 0) {
    can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_TEMPERATURE));
    can_->send_rtr(make_arb_id(right_axis_id_, cmd::GET_TEMPERATURE));
    can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_VBUS_VOLTAGE));
  }
}

// ── Read CAN messages ──

void ODriveCANNode::read_can_messages() {
  struct can_frame frame;

  // Drain all pending frames (non-blocking, 1ms timeout)
  while (can_->recv(frame, 1)) {
    uint8_t node_id = get_node_id(frame.can_id & CAN_SFF_MASK);
    uint8_t cmd_id  = get_cmd_id(frame.can_id & CAN_SFF_MASK);

    AxisData* axis = nullptr;
    if (node_id == left_axis_id_) {
      axis = &left_;
    } else if (node_id == right_axis_id_) {
      axis = &right_;
    } else {
      continue;
    }

    switch (cmd_id) {
      case cmd::HEARTBEAT: {
        auto hb = HeartbeatMsg::parse(frame.data);
        axis->state = hb.axis_state;
        axis->errors = hb.active_errors;
        axis->heartbeat_received = true;
        if (hb.active_errors != 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "ODrive node %d errors: 0x%08X", node_id, hb.active_errors);
        }
        break;
      }
      case cmd::GET_ENCODER_ESTIMATES: {
        auto enc = EncoderMsg::parse(frame.data);
        if (!enc.valid) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "ODrive node %d: NaN/Inf in encoder feedback — ignoring", node_id);
          break;
        }
        double sign = (node_id == left_axis_id_) ? left_sign_ : right_sign_;
        axis->position = enc.position * sign;
        axis->velocity = enc.velocity * sign;
        break;
      }
      case cmd::GET_TEMPERATURE: {
        auto temp = TemperatureMsg::parse(frame.data);
        axis->fet_temp = temp.fet_temperature;
        axis->motor_temp = temp.motor_temperature;
        check_temperature(*axis);
        break;
      }
      case cmd::GET_VBUS_VOLTAGE: {
        auto vbus = VbusMsg::parse(frame.data);
        bus_voltage_ = vbus.voltage;
        bus_current_ = vbus.current;
        break;
      }
      default:
        break;
    }
  }
}

// ── Odometry integration ──

void ODriveCANNode::integrate_odometry() {
  auto now = this->now();
  double dt = (now - last_odom_time_).seconds();
  last_odom_time_ = now;

  if (dt <= 0.0 || dt > 1.0) return;

  if (!odom_initialized_) {
    left_.prev_position = left_.position;
    right_.prev_position = right_.position;
    odom_initialized_ = true;
    return;
  }

  // Filter encoder velocities (EMA low-pass — smooths noise for twist + rotation detection)
  left_vel_filtered_  = velocity_filter_alpha_ * left_.velocity
                      + (1.0 - velocity_filter_alpha_) * left_vel_filtered_;
  right_vel_filtered_ = velocity_filter_alpha_ * right_.velocity
                      + (1.0 - velocity_filter_alpha_) * right_vel_filtered_;

  // Compute wheel travel in meters (position is in motor turns; divide by gear_ratio for wheel turns)
  double delta_left  = (left_.position - left_.prev_position) / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;
  double delta_right = (right_.position - right_.prev_position) / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;

  left_.prev_position = left_.position;
  right_.prev_position = right_.position;

  // Detect pure rotation: wheels spinning opposite directions with similar magnitude.
  // During pure rotation, encoder noise asymmetry creates false translation (delta_s ≠ 0).
  // Fix: force symmetric deltas so translation cancels exactly.
  double v_sum = left_vel_filtered_ + right_vel_filtered_;
  double v_diff = std::abs(left_vel_filtered_) + std::abs(right_vel_filtered_);
  pure_rotation_ = (v_diff > 0.1) && (std::abs(v_sum) / v_diff < 0.15);
  if (pure_rotation_) {
    double avg_mag = (std::abs(delta_left) + std::abs(delta_right)) / 2.0;
    double sign_l = (delta_left >= 0.0) ? 1.0 : -1.0;
    double sign_r = (delta_right >= 0.0) ? 1.0 : -1.0;
    delta_left  = avg_mag * sign_l;
    delta_right = avg_mag * sign_r;
  }

  // Differential drive kinematics
  double delta_s     = (delta_left + delta_right) / 2.0;
  double delta_theta = (delta_right - delta_left) / track_width_;

  // Mid-angle integration (better than Euler for arcs)
  double mid_theta = odom_theta_ + delta_theta / 2.0;
  odom_x_     += delta_s * std::cos(mid_theta);
  odom_y_     += delta_s * std::sin(mid_theta);
  odom_theta_ += delta_theta;
}

// ── Caster disturbance tracking ──

void ODriveCANNode::update_caster_disturbance(double v_linear, double v_angular, double dt) {
  if (!caster_enable_compensation_ || dt <= 0.0) return;

  bool direction_change = false;

  // Check linear velocity sign change (forward <-> backward)
  constexpr double vel_noise_floor = 0.02;  // m/s
  if (std::abs(prev_v_linear_) > vel_noise_floor &&
      std::abs(v_linear) > vel_noise_floor &&
      prev_v_linear_ * v_linear < 0.0) {
    direction_change = true;
  }

  // Check angular velocity sign change (left <-> right turn)
  if (std::abs(prev_v_angular_) > vel_noise_floor &&
      std::abs(v_angular) > vel_noise_floor &&
      prev_v_angular_ * v_angular < 0.0) {
    direction_change = true;
  }

  // Check angular acceleration (abrupt turn initiation from straight line)
  double angular_accel = (v_angular - prev_v_angular_) / dt;
  if (std::abs(angular_accel) > caster_angular_accel_threshold_) {
    direction_change = true;
  }

  prev_v_linear_ = v_linear;
  prev_v_angular_ = v_angular;

  // Sustained pure rotation also disturbs caster alignment
  // (caster wheels scrub sideways during continuous in-place rotation)
  constexpr double sustained_rotation_threshold = 0.3;  // rad/s
  if (std::abs(v_angular) > sustained_rotation_threshold && std::abs(v_linear) < 0.05) {
    caster_disturbance_level_ = std::max(caster_disturbance_level_,
                                          caster_covariance_multiplier_ * 0.3);
  }

  // On direction change, reset disturbance to peak
  if (direction_change) {
    caster_disturbance_level_ = caster_covariance_multiplier_;
  }

  // Exponential decay
  caster_disturbance_level_ *= std::exp(-dt / caster_settling_tau_);

  // Clamp to zero when negligible
  if (caster_disturbance_level_ < 0.01) {
    caster_disturbance_level_ = 0.0;
  }
}

// ── Publish odometry ──

void ODriveCANNode::publish_odometry() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  // Pose
  msg.pose.pose.position.x = odom_x_;
  msg.pose.pose.position.y = odom_y_;
  msg.pose.pose.position.z = 0.0;

  // Quaternion from yaw
  double half_yaw = odom_theta_ / 2.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(half_yaw);
  msg.pose.pose.orientation.w = std::cos(half_yaw);

  // Twist from filtered velocities (smoother signal for EKF)
  double v_left  = left_vel_filtered_ / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;   // m/s
  double v_right = right_vel_filtered_ / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;  // m/s
  double v_linear = (v_left + v_right) / 2.0;
  double v_angular = (v_right - v_left) / track_width_;
  msg.twist.twist.linear.x  = v_linear;
  msg.twist.twist.angular.z = v_angular;

  // Update caster disturbance model (direction change detection + decay)
  double caster_dt = 1.0 / static_cast<double>(publish_rate_hz_);
  update_caster_disturbance(v_linear, v_angular, caster_dt);

  // Pose covariance — grows with angular velocity, caster disturbance, and wheel slip
  double base_xy_cov = (pure_rotation_ || wheel_slip_active_) ? 0.05 : 0.01;
  double base_yaw_cov = 0.03;
  double angular_factor = 1.0 + 5.0 * std::abs(v_angular);  // 5x per rad/s (was 2x)
  double caster_factor = 1.0 + caster_disturbance_level_;
  double total_cov_factor = angular_factor * caster_factor;
  msg.pose.covariance[0]  = base_xy_cov * total_cov_factor;    // x
  msg.pose.covariance[7]  = base_xy_cov * total_cov_factor;    // y
  msg.pose.covariance[35] = base_yaw_cov * total_cov_factor;   // yaw

  // Twist covariance — velocity-dependent + caster disturbance
  double speed_factor = 1.0 + std::abs(v_linear);
  msg.twist.covariance[0]  = 0.01 * speed_factor * caster_factor;
  msg.twist.covariance[35] = 0.03 * speed_factor * caster_factor;

  pub_odom_->publish(msg);
}

// ── Publish joint states ──

void ODriveCANNode::publish_joint_states() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();

  msg.name = {"left_wheel_joint", "right_wheel_joint"};

  // Position in radians (turns * 2π)
  msg.position = {
    left_.position * 2.0 * M_PI,
    right_.position * 2.0 * M_PI
  };

  // Velocity in rad/s (turns/s * 2π)
  msg.velocity = {
    left_.velocity * 2.0 * M_PI,
    right_.velocity * 2.0 * M_PI
  };

  pub_joint_->publish(msg);
}

// ── cmd_vel callback ──

void ODriveCANNode::on_cmd_vel(const geometry_msgs::msg::Twist& msg) {
  if (e_stop_active_) return;
  if (!motors_armed()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "cmd_vel received but motors not armed (left=%d, right=%d)",
      left_.state, right_.state);
    return;
  }

  last_cmd_vel_time_ = this->now();

  double linear_x  = msg.linear.x;
  double angular_z = msg.angular.z;
  last_linear_cmd_  = linear_x;
  last_angular_cmd_ = angular_z;

  // Wheel slip detection: if encoder velocities diverge during straight-line command,
  // reduce command to prevent one wheel from exceeding friction coefficient.
  double vel_diff = std::abs(left_vel_filtered_ - right_vel_filtered_);
  bool commanding_straight = std::abs(angular_z) < 0.05 && std::abs(linear_x) > 0.01;
  if (commanding_straight && vel_diff > slip_velocity_threshold_) {
    if (!wheel_slip_active_) {
      wheel_slip_active_ = true;
      slip_start_time_ = this->now();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Wheel slip detected (vel_diff=%.2f turns/s), reducing speed", vel_diff);
    }
    linear_x *= slip_reduction_factor_;
  } else if (wheel_slip_active_) {
    double elapsed_ms = (this->now() - slip_start_time_).seconds() * 1000.0;
    if (elapsed_ms > slip_cooldown_ms_) {
      wheel_slip_active_ = false;
    } else {
      linear_x *= slip_reduction_factor_;
    }
  }

  // Differential drive inverse kinematics: m/s → motor turns/s (multiply by gear_ratio)
  double v_left  = (linear_x - angular_z * track_width_ / 2.0) / (wheel_radius_ * 2.0 * M_PI) * gear_ratio_;
  double v_right = (linear_x + angular_z * track_width_ / 2.0) / (wheel_radius_ * 2.0 * M_PI) * gear_ratio_;

  // Apply shaping: inversion → scale → accel limit → min effective → torque_ff
  float left_cmd  = apply_wheel_shaping(static_cast<float>(v_left), left_.prev_cmd, left_sign_ * left_scale_);
  float right_cmd = apply_wheel_shaping(static_cast<float>(v_right), right_.prev_cmd, right_sign_ * right_scale_);

  last_left_target_  = left_cmd;
  last_right_target_ = right_cmd;
  zero_cmd_active_ = (left_cmd == 0.0f && right_cmd == 0.0f);

  float left_ff  = (std::abs(left_cmd) > 0.001f) ? std::copysign(stiction_torque_ff_, left_cmd) : 0.0f;
  float right_ff = (std::abs(right_cmd) > 0.001f) ? std::copysign(stiction_torque_ff_, right_cmd) : 0.0f;

  send_velocity_with_ff(left_axis_id_, left_cmd, left_ff);
  send_velocity_with_ff(right_axis_id_, right_cmd, right_ff);
}

// ── E-stop callback ──

void ODriveCANNode::on_e_stop(const std_msgs::msg::Bool& msg) {
  if (msg.data && !e_stop_active_) {
    RCLCPP_WARN(get_logger(), "E-STOP ACTIVATED");
    e_stop_active_ = true;
    left_.prev_cmd  = 0.0f;
    right_.prev_cmd = 0.0f;
    last_left_target_  = 0.0f;
    last_right_target_ = 0.0f;
    zero_cmd_active_ = true;
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
  } else if (!msg.data && e_stop_active_) {
    RCLCPP_INFO(get_logger(), "E-stop released");
    e_stop_active_ = false;
  }
}

// ── Wheel command shaping ──

float ODriveCANNode::apply_wheel_shaping(float target, float& prev_cmd, double sign_and_scale) {
  // Zero-command bypass: release → exact zero, no accel-limiter creep, no min_effective snap
  if (std::abs(target) < zero_vel_epsilon_) {
    prev_cmd = 0.0f;
    return 0.0f;
  }

  float vel = target * static_cast<float>(sign_and_scale);

  // Asymmetric accel limiter: decel is faster than accel for responsive stopping
  float dt = 1.0f / static_cast<float>(publish_rate_hz_);
  bool decelerating = (std::abs(vel) < std::abs(prev_cmd));
  float max_dv = (decelerating ? max_wheel_decel_ : max_wheel_accel_) * dt;
  vel = prev_cmd + std::clamp(vel - prev_cmd, -max_dv, max_dv);

  // Min effective velocity (stiction compensation) — disabled by default (min_effective_vel=0)
  if (min_effective_vel_ > 0.0f && std::abs(vel) > 0.001f && std::abs(vel) < min_effective_vel_) {
    vel = std::copysign(min_effective_vel_, vel);
  }

  prev_cmd = vel;
  return vel;
}

// ── CAN send helpers ──

void ODriveCANNode::send_velocity(uint8_t node_id, float vel_turns_per_s) {
  send_velocity_with_ff(node_id, vel_turns_per_s, 0.0f);
}

void ODriveCANNode::send_velocity_with_ff(uint8_t node_id, float vel_turns_per_s, float torque_ff) {
  if (!can_ || !can_->is_open()) return;
  uint8_t data[8];
  pack_velocity(data, vel_turns_per_s, torque_ff);
  can_->send(make_arb_id(node_id, cmd::SET_INPUT_VEL), data, 8);
}

void ODriveCANNode::send_axis_state(uint8_t node_id, AxisState state) {
  if (!can_ || !can_->is_open()) return;
  uint8_t data[8];
  pack_axis_state(data, state);
  can_->send(make_arb_id(node_id, cmd::SET_AXIS_STATE), data, 8);
}

void ODriveCANNode::stop_motors() {
  RCLCPP_INFO(get_logger(), "Stopping motors...");
  send_velocity(left_axis_id_, 0.0f);
  send_velocity(right_axis_id_, 0.0f);
  send_axis_state(left_axis_id_, AxisState::IDLE);
  send_axis_state(right_axis_id_, AxisState::IDLE);
}

// ── Motor readiness ──

bool ODriveCANNode::motors_armed() const {
  return left_.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL) &&
         right_.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL);
}

void ODriveCANNode::publish_motor_state() {
  std_msgs::msg::String msg;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    R"({"left_state":%d,"right_state":%d,"left_errors":%u,"right_errors":%u,"armed":%s,)"
    R"("bus_voltage":%.2f,"bus_current":%.2f,)"
    R"("left_fet_temp":%.1f,"left_motor_temp":%.1f,)"
    R"("right_fet_temp":%.1f,"right_motor_temp":%.1f,"thermal_state":"%s"})",
    left_.state, right_.state, left_.errors, right_.errors,
    motors_armed() ? "true" : "false",
    bus_voltage_, bus_current_,
    static_cast<double>(left_.fet_temp), static_cast<double>(left_.motor_temp),
    static_cast<double>(right_.fet_temp), static_cast<double>(right_.motor_temp),
    thermal_state_.c_str());
  msg.data = buf;
  pub_motor_state_->publish(msg);
}

void ODriveCANNode::publish_drive_debug() {
  auto elapsed_ms = (this->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
  bool cmd_valid = (elapsed_ms <= cmd_vel_timeout_ms_);

  std_msgs::msg::String msg;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    R"({"cmd_linear":%.4f,"cmd_angular":%.4f,"left_target":%.4f,"right_target":%.4f,)"
    R"("left_meas":%.4f,"right_meas":%.4f,"armed":%s,"e_stop":%s,"cmd_valid":%s,"zero_cmd":%s,)"
    R"("caster_disturbance":%.4f})",
    last_linear_cmd_, last_angular_cmd_,
    static_cast<double>(last_left_target_),
    static_cast<double>(last_right_target_),
    static_cast<double>(left_.velocity),
    static_cast<double>(right_.velocity),
    motors_armed() ? "true" : "false",
    e_stop_active_ ? "true" : "false",
    cmd_valid ? "true" : "false",
    zero_cmd_active_ ? "true" : "false",
    caster_disturbance_level_);
  msg.data = buf;
  pub_drive_debug_->publish(msg);
}

void ODriveCANNode::on_motor_enable(const std_msgs::msg::Bool& msg) {
  if (msg.data) {
    RCLCPP_INFO(get_logger(), "Enabling motors → CLOSED_LOOP_CONTROL");
    send_axis_state(left_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
    send_axis_state(right_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
  } else {
    RCLCPP_INFO(get_logger(), "Disabling motors → IDLE");
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
    send_axis_state(left_axis_id_, AxisState::IDLE);
    send_axis_state(right_axis_id_, AxisState::IDLE);
  }
}

}  // namespace agv_odrive
