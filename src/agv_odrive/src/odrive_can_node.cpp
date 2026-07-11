#include "agv_odrive/odrive_can_node.hpp"

#include <algorithm>
#include <cmath>
#include <linux/can.h>

#include "agv_odrive/kinematics.hpp"

namespace agv_odrive {

ODriveCANNode::ODriveCANNode() : Node("agv_odrive_node") {
  // -- Declare parameters --
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("left_axis_id", 0);
  this->declare_parameter("right_axis_id", 1);
  // Calibrated physical constants have NO in-code defaults — they must come
  // from YAML (config/odrive_params.yaml) so a missing file aborts startup
  // instead of silently driving with wrong kinematics.
  this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("track_width", rclcpp::PARAMETER_DOUBLE);
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
  // gear_ratio = motor_turns / wheel_turns. Use 1.0 if ODrive firmware already
  // has gear_ratio configured, 10.0 for a 10:1 planetary gearbox with raw
  // encoder feedback. Required from YAML (see wheel_radius above).
  this->declare_parameter("gear_ratio", rclcpp::PARAMETER_DOUBLE);
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
  // Feedback watchdog: fault if no heartbeat/encoder frame arrives within this
  // window (ODrive heartbeat period is 100 ms → default = 3 missed). 0 disables.
  this->declare_parameter("feedback_timeout_ms", 300);

  // -- Read parameters --
  can_interface_ = this->get_parameter("can_interface").as_string();
  left_axis_id_ = static_cast<uint8_t>(this->get_parameter("left_axis_id").as_int());
  right_axis_id_ = static_cast<uint8_t>(this->get_parameter("right_axis_id").as_int());
  try {
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    gear_ratio_ = this->get_parameter("gear_ratio").as_double();
  } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(get_logger(),
      "wheel_radius, track_width and gear_ratio have no in-code defaults — "
      "provide the calibrated values via YAML (config/odrive_params.yaml)");
    throw;
  }
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
  feedback_timeout_ms_ = static_cast<int>(this->get_parameter("feedback_timeout_ms").as_int());

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
  if (publish_rate_hz_ < 1 || publish_rate_hz_ > 1000) {
    RCLCPP_FATAL(get_logger(), "publish_rate_hz must be in [1, 1000], got %d", publish_rate_hz_);
    throw std::runtime_error("Invalid publish_rate_hz");
  }
  if (feedback_timeout_ms_ < 0) {
    RCLCPP_FATAL(get_logger(), "feedback_timeout_ms must be >= 0 (0 disables), got %d",
                 feedback_timeout_ms_);
    throw std::runtime_error("Invalid feedback_timeout_ms");
  }

  // -- Kinematics boot diagnostic (CRITICAL-02-02 closure) --
  // Log the live kinematics at every boot so the operator can cross-check
  // against the geometry SSOT (agv_description/config/robot_geometry.yaml)
  // and, on any doubt, against ODrive NVRAM via odrivetool
  // (docs/calibration/odrive_nvram_dump_procedure.md). If ROS gear_ratio
  // != 1.0 AND ODrive NVRAM also gears, motor turns are double-counted —
  // verify before deployment.
  RCLCPP_INFO(get_logger(),
              "Kinematics SSOT: wheel_radius=%.4fm, track_width=%.4fm, gear_ratio=%.2f. "
              "Cross-check against robot_geometry.yaml and ODrive NVRAM "
              "(encoder.cpr, motor.config.pole_pairs) per "
              "docs/calibration/odrive_nvram_dump_procedure.md.",
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
  left_.last_feedback = this->now();
  right_.last_feedback = this->now();

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

void ODriveCANNode::check_temperature(AxisData& axis) {
  // Each sensor is compared against its OWN limit (FET vs max_fet_temp,
  // motor vs max_motor_temp). A critical axis stays latched critical until it
  // cools below the warning thresholds — hysteresis that prevents re-arm /
  // shutdown flapping near the limit.
  const bool critical = axis.fet_temp > max_fet_temp_ + critical_temp_offset_ ||
                        axis.motor_temp > max_motor_temp_ + critical_temp_offset_;
  const bool warning = axis.fet_temp > max_fet_temp_ || axis.motor_temp > max_motor_temp_;

  if (critical) {
    axis.thermal_level = 2;
  } else if (warning) {
    if (axis.thermal_level < 2) axis.thermal_level = 1;
  } else {
    axis.thermal_level = 0;
  }

  // Report and act on the worst axis, not the most recently updated one —
  // otherwise a healthy axis's frame would overwrite the hot axis's verdict.
  const int worst = std::max(left_.thermal_level, right_.thermal_level);
  if (worst >= 2) {
    thermal_state_ = "critical";
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "CRITICAL: Temperature limit exceeded (FET=%.1f Motor=%.1f), disabling motors",
      static_cast<double>(axis.fet_temp), static_cast<double>(axis.motor_temp));
    stop_motors();
  } else if (worst == 1) {
    thermal_state_ = "warning";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Temperature warning: FET=%.1f (limit %.1f) Motor=%.1f (limit %.1f)",
      static_cast<double>(axis.fet_temp), max_fet_temp_,
      static_cast<double>(axis.motor_temp), max_motor_temp_);
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
    // Fresh socket: restart the feedback deadline and re-latch the odometry
    // baseline from the next encoder frames (positions may have jumped while
    // the bus was down).
    left_.last_feedback = this->now();
    right_.last_feedback = this->now();
    left_.encoder_seen = false;
    right_.encoder_seen = false;
    odom_initialized_ = false;
  }

  // Read all pending CAN messages
  read_can_messages();

  // Feedback watchdog: heartbeats arrive every 100 ms and encoder replies at
  // 50 Hz while the bus and ODrive are alive. A SocketCAN fd stays "open"
  // when the interface drops or the ODrive powers off, so socket state alone
  // cannot detect a dead bus — frame freshness can.
  if (feedback_timeout_ms_ > 0) {
    const auto fb_now = this->now();
    const auto left_age_ms = (fb_now - left_.last_feedback).nanoseconds() / 1000000;
    const auto right_age_ms = (fb_now - right_.last_feedback).nanoseconds() / 1000000;
    const bool stale = left_age_ms > feedback_timeout_ms_ || right_age_ms > feedback_timeout_ms_;
    if (stale && feedback_ok_) {
      feedback_ok_ = false;
      // Stop trusting the frozen encoder data: zero the reported velocity and
      // re-arm the odometry first-sample gate so the pose re-latches cleanly
      // when feedback returns (the ODrive may have been power-cycled,
      // resetting Pos_Estimate).
      left_.velocity = 0.0f;
      right_.velocity = 0.0f;
      left_vel_filtered_ = 0.0;
      right_vel_filtered_ = 0.0;
      left_.encoder_seen = false;
      right_.encoder_seen = false;
      odom_initialized_ = false;
      RCLCPP_ERROR(get_logger(),
        "ODrive feedback lost (left %lld ms, right %lld ms > %d ms) — "
        "reporting motors disarmed and zeroing measured twist",
        static_cast<long long>(left_age_ms), static_cast<long long>(right_age_ms),
        feedback_timeout_ms_);
    } else if (!stale && !feedback_ok_) {
      feedback_ok_ = true;
      RCLCPP_INFO(get_logger(), "ODrive feedback restored");
    }
  }

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
    // RTR polls carry no payload (e.g. our own encoder requests looped back,
    // or another local process polling the same bus) — parsing one would
    // inject position/velocity 0.0. Drop them before dispatching.
    if (frame.can_id & CAN_RTR_FLAG) continue;

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
        if (frame.can_dlc < 7) break;  // errors(4) + state(1) + result(1) + done(1)
        auto hb = HeartbeatMsg::parse(frame.data);
        axis->state = hb.axis_state;
        axis->errors = hb.active_errors;
        axis->last_feedback = this->now();
        if (hb.active_errors != 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "ODrive node %d errors: 0x%08X", node_id, hb.active_errors);
        }
        break;
      }
      case cmd::GET_ENCODER_ESTIMATES: {
        if (frame.can_dlc < 8) break;  // two float32
        auto enc = EncoderMsg::parse(frame.data);
        if (!enc.valid) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "ODrive node %d: NaN/Inf in encoder feedback — ignoring", node_id);
          break;
        }
        double sign = (node_id == left_axis_id_) ? left_sign_ : right_sign_;
        axis->position = enc.position * sign;
        axis->velocity = enc.velocity * sign;
        axis->encoder_seen = true;
        axis->last_feedback = this->now();
        break;
      }
      case cmd::GET_TEMPERATURE: {
        if (frame.can_dlc < 8) break;
        auto temp = TemperatureMsg::parse(frame.data);
        axis->fet_temp = temp.fet_temperature;
        axis->motor_temp = temp.motor_temperature;
        check_temperature(*axis);
        break;
      }
      case cmd::GET_VBUS_VOLTAGE: {
        if (frame.can_dlc < 8) break;
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
    // Latch the baseline only once both axes have reported a real encoder
    // frame — ODrive Pos_Estimate persists across driver restarts, so seeding
    // from the default 0.0 would produce a huge spurious first delta.
    if (!left_.encoder_seen || !right_.encoder_seen) return;
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

  // Compute wheel travel in meters (position is in motor turns)
  double delta_left = kinematics::motor_turns_to_meters(
      left_.position - left_.prev_position, gear_ratio_, wheel_radius_);
  double delta_right = kinematics::motor_turns_to_meters(
      right_.position - right_.prev_position, gear_ratio_, wheel_radius_);

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

  // Differential drive forward kinematics (mid-angle integration)
  auto d = kinematics::wheel_deltas_to_odom(delta_left, delta_right, track_width_, odom_theta_);
  odom_x_     += d.dx;
  odom_y_     += d.dy;
  odom_theta_ += d.dtheta;
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
  double v_left  = kinematics::motor_turns_to_meters(left_vel_filtered_, gear_ratio_, wheel_radius_);   // m/s
  double v_right = kinematics::motor_turns_to_meters(right_vel_filtered_, gear_ratio_, wheel_radius_);  // m/s
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
  // Feedback loss: encoder data is stale, so the (zeroed) twist and frozen
  // pose must not be trusted — inflate covariance so the EKF leans on the IMU.
  double feedback_factor = feedback_ok_ ? 1.0 : 100.0;
  double total_cov_factor = angular_factor * caster_factor * feedback_factor;
  msg.pose.covariance[0]  = base_xy_cov * total_cov_factor;    // x
  msg.pose.covariance[7]  = base_xy_cov * total_cov_factor;    // y
  msg.pose.covariance[35] = base_yaw_cov * total_cov_factor;   // yaw

  // Twist covariance — velocity-dependent + caster disturbance
  double speed_factor = 1.0 + std::abs(v_linear);
  msg.twist.covariance[0]  = 0.01 * speed_factor * caster_factor * feedback_factor;
  msg.twist.covariance[35] = 0.03 * speed_factor * caster_factor * feedback_factor;

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

  const auto cmd_now = this->now();
  // Accel limiting must use the real inter-command interval: upstream
  // publishers range from ~10 Hz teleop to 50 Hz Nav2, and assuming a fixed
  // rate would make the effective accel limit scale with the sender's rate.
  // Clamped so a long gap cannot authorize one huge velocity step.
  const float shaping_dt = static_cast<float>(
      std::clamp((cmd_now - last_cmd_vel_time_).seconds(), 0.001, 0.1));
  last_cmd_vel_time_ = cmd_now;

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

  // Differential drive inverse kinematics: m/s → motor turns/s
  auto wheels = kinematics::cmd_vel_to_wheels(linear_x, angular_z,
                                              wheel_radius_, track_width_, gear_ratio_);

  // Apply shaping: inversion → scale → accel limit → min effective → torque_ff
  float left_cmd  = apply_wheel_shaping(static_cast<float>(wheels.left), left_.prev_cmd,
                                        left_sign_ * left_scale_, shaping_dt);
  float right_cmd = apply_wheel_shaping(static_cast<float>(wheels.right), right_.prev_cmd,
                                        right_sign_ * right_scale_, shaping_dt);

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

float ODriveCANNode::apply_wheel_shaping(float target, float& prev_cmd, double sign_and_scale,
                                         float dt) {
  // Zero-command bypass: release → exact zero, no accel-limiter creep, no min_effective snap
  if (std::abs(target) < zero_vel_epsilon_) {
    prev_cmd = 0.0f;
    return 0.0f;
  }

  float vel = target * static_cast<float>(sign_and_scale);

  // Asymmetric accel limiter: decel is faster than accel for responsive stopping
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
  send_frame(make_arb_id(node_id, cmd::SET_INPUT_VEL), data);
}

void ODriveCANNode::send_axis_state(uint8_t node_id, AxisState state) {
  if (!can_ || !can_->is_open()) return;
  uint8_t data[8];
  pack_axis_state(data, state);
  send_frame(make_arb_id(node_id, cmd::SET_AXIS_STATE), data);
}

void ODriveCANNode::send_frame(uint32_t arb_id, const uint8_t* data) {
  if (can_->send(arb_id, data, 8)) {
    can_send_failures_ = 0;
    return;
  }
  // Consecutive failures are a bus-down signal (ENETDOWN, or tx queue full
  // because no node ACKs). The feedback watchdog handles the state change;
  // this makes the failures visible instead of silently dropped.
  ++can_send_failures_;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "CAN send failed (%d consecutive) on %s", can_send_failures_, can_interface_.c_str());
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
  // Stale feedback means the axis states are unknown — report disarmed so
  // cmd_vel is not forwarded into a dead bus and the operator sees the fault.
  return feedback_ok_ &&
         left_.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL) &&
         right_.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL);
}

void ODriveCANNode::publish_motor_state() {
  std_msgs::msg::String msg;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    R"({"left_state":%d,"right_state":%d,"left_errors":%u,"right_errors":%u,"armed":%s,)"
    R"("bus_voltage":%.2f,"bus_current":%.2f,)"
    R"("left_fet_temp":%.1f,"left_motor_temp":%.1f,)"
    R"("right_fet_temp":%.1f,"right_motor_temp":%.1f,)"
    R"("feedback_ok":%s,"thermal_state":"%s"})",
    left_.state, right_.state, left_.errors, right_.errors,
    motors_armed() ? "true" : "false",
    bus_voltage_, bus_current_,
    static_cast<double>(left_.fet_temp), static_cast<double>(left_.motor_temp),
    static_cast<double>(right_.fet_temp), static_cast<double>(right_.motor_temp),
    feedback_ok_ ? "true" : "false",
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
    if (thermal_state_ == "critical") {
      RCLCPP_WARN(get_logger(),
        "motor_enable rejected: thermal state is critical (must cool below warning limits)");
      return;
    }
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
