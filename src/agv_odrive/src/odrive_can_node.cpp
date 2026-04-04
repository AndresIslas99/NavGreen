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
  this->declare_parameter("invert_left", false);
  this->declare_parameter("invert_right", true);
  this->declare_parameter("left_scale", 1.0);
  this->declare_parameter("right_scale", 1.0);
  this->declare_parameter("min_effective_vel", 0.0);
  this->declare_parameter("stiction_torque_ff", 0.0);
  this->declare_parameter("max_wheel_accel", 1.0);
  this->declare_parameter("zero_vel_epsilon", 0.03);
  // gear_ratio = motor_turns / wheel_turns. Set to 1.0 if ODrive firmware already
  // has gear_ratio configured. Set to 10.0 for a 10:1 planetary gearbox with raw
  // encoder feedback.
  this->declare_parameter("gear_ratio", 1.0);

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
  zero_vel_epsilon_ = static_cast<float>(this->get_parameter("zero_vel_epsilon").as_double());
  gear_ratio_ = this->get_parameter("gear_ratio").as_double();

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

  timer_motor_state_ = this->create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&ODriveCANNode::publish_motor_state, this));

  timer_debug_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&ODriveCANNode::publish_drive_debug, this));

  RCLCPP_INFO(get_logger(), "ODrive CAN node started on %s (left=%d, right=%d)",
              can_interface_.c_str(), left_axis_id_, right_axis_id_);
  RCLCPP_INFO(get_logger(), "wheel_radius=%.4f m, track_width=%.4f m, rate=%d Hz, gear_ratio=%.2f",
              wheel_radius_, track_width_, publish_rate_hz_, gear_ratio_);
  RCLCPP_INFO(get_logger(), "invert_left=%s, invert_right=%s",
              invert_left_ ? "true" : "false", invert_right_ ? "true" : "false");
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

// ── Main loop (50 Hz) ──

void ODriveCANNode::main_loop() {
  // Retry CAN if not connected
  if (!can_ || !can_->is_open()) {
    if (!init_can()) return;
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

  // Compute wheel travel in meters (position is in motor turns; divide by gear_ratio for wheel turns)
  double delta_left  = (left_.position - left_.prev_position) / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;
  double delta_right = (right_.position - right_.prev_position) / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;

  left_.prev_position = left_.position;
  right_.prev_position = right_.position;

  // Differential drive kinematics
  double delta_s     = (delta_left + delta_right) / 2.0;
  double delta_theta = (delta_right - delta_left) / track_width_;

  // Mid-angle integration (better than Euler for arcs)
  double mid_theta = odom_theta_ + delta_theta / 2.0;
  odom_x_     += delta_s * std::cos(mid_theta);
  odom_y_     += delta_s * std::sin(mid_theta);
  odom_theta_ += delta_theta;
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

  // Twist (from current encoder velocities — motor turns/s divided by gear_ratio for wheel turns/s)
  double v_left  = left_.velocity / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;   // m/s
  double v_right = right_.velocity / gear_ratio_ * wheel_radius_ * 2.0 * M_PI;  // m/s
  double v_linear = (v_left + v_right) / 2.0;
  double v_angular = (v_right - v_left) / track_width_;
  msg.twist.twist.linear.x  = v_linear;
  msg.twist.twist.angular.z = v_angular;

  // Pose covariance — grows with angular velocity (turns are less accurate)
  double base_xy_cov = 0.01;
  double base_yaw_cov = 0.03;
  double angular_factor = 1.0 + 2.0 * std::abs(v_angular);  // higher when turning
  msg.pose.covariance[0]  = base_xy_cov * angular_factor;    // x
  msg.pose.covariance[7]  = base_xy_cov * angular_factor;    // y
  msg.pose.covariance[35] = base_yaw_cov * angular_factor;   // yaw

  // Twist covariance — velocity-dependent (less certain at higher speeds)
  double speed_factor = 1.0 + std::abs(v_linear);
  msg.twist.covariance[0]  = 0.01 * speed_factor;
  msg.twist.covariance[35] = 0.03 * speed_factor;

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

  // Accel limiter
  float dt = 1.0f / static_cast<float>(publish_rate_hz_);
  float max_dv = max_wheel_accel_ * dt;
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
    R"("right_fet_temp":%.1f,"right_motor_temp":%.1f})",
    left_.state, right_.state, left_.errors, right_.errors,
    motors_armed() ? "true" : "false",
    bus_voltage_, bus_current_,
    static_cast<double>(left_.fet_temp), static_cast<double>(left_.motor_temp),
    static_cast<double>(right_.fet_temp), static_cast<double>(right_.motor_temp));
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
    R"("left_meas":%.4f,"right_meas":%.4f,"armed":%s,"e_stop":%s,"cmd_valid":%s,"zero_cmd":%s})",
    last_linear_cmd_, last_angular_cmd_,
    static_cast<double>(last_left_target_),
    static_cast<double>(last_right_target_),
    static_cast<double>(left_.velocity),
    static_cast<double>(right_.velocity),
    motors_armed() ? "true" : "false",
    e_stop_active_ ? "true" : "false",
    cmd_valid ? "true" : "false",
    zero_cmd_active_ ? "true" : "false");
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
