#include "agv_safety/cmd_vel_gate.hpp"

#include <algorithm>

namespace agv_safety {

namespace {
double clamp_signed(double v, double limit) {
  if (limit <= 0.0) return v;
  return std::max(-limit, std::min(limit, v));
}
}  // namespace

geometry_msgs::msg::Twist apply_gate(const GateInputs& in) {
  geometry_msgs::msg::Twist out;
  if (!in.safety_ok || in.hardware_estop) {
    return out;  // zero velocity
  }
  out = in.input_cmd;
  out.linear.x = clamp_signed(out.linear.x, in.max_linear);
  out.linear.y = clamp_signed(out.linear.y, in.max_linear);
  out.angular.z = clamp_signed(out.angular.z, in.max_angular);
  return out;
}

CmdVelGateNode::CmdVelGateNode() : rclcpp::Node("cmd_vel_gate") {
  declare_parameters();
  safety_timeout_s_ = get_parameter("safety_timeout_s").as_double();
  max_linear_ = get_parameter("max_linear").as_double();
  max_angular_ = get_parameter("max_angular").as_double();

  pub_output_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_out", rclcpp::QoS(10).reliable());

  sub_input_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", rclcpp::QoS(10).reliable(),
      std::bind(&CmdVelGateNode::on_input, this, std::placeholders::_1));

  sub_safety_ = create_subscription<agv_interfaces::msg::SafetyStatus>(
      "safety_status", rclcpp::QoS(10).reliable(),
      std::bind(&CmdVelGateNode::on_safety, this, std::placeholders::_1));

  sub_hw_estop_ = create_subscription<std_msgs::msg::Bool>(
      "hardware_estop", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&CmdVelGateNode::on_hardware_estop, this, std::placeholders::_1));

  // Watchdog: if no SafetyStatus arrives within safety_timeout_s, treat
  // safety_ok as false. This catches a crashed safety_supervisor.
  const auto period = std::chrono::milliseconds(
      static_cast<int>(safety_timeout_s_ * 1000.0 / 2.0));
  watchdog_ = create_wall_timer(period, std::bind(&CmdVelGateNode::on_safety_timeout, this));

  RCLCPP_INFO(get_logger(),
              "cmd_vel_gate up: max_linear=%.2f m/s, max_angular=%.2f rad/s, "
              "safety_timeout=%.2fs",
              max_linear_, max_angular_, safety_timeout_s_);
}

void CmdVelGateNode::declare_parameters() {
  declare_parameter<double>("max_linear", 0.5);
  declare_parameter<double>("max_angular", 1.5);
  declare_parameter<double>("safety_timeout_s", 0.5);
}

void CmdVelGateNode::on_input(const geometry_msgs::msg::Twist& msg) {
  GateInputs in;
  in.input_cmd = msg;
  in.safety_ok = safety_ok_;
  in.hardware_estop = hardware_estop_;
  in.max_linear = max_linear_;
  in.max_angular = max_angular_;
  pub_output_->publish(apply_gate(in));
}

void CmdVelGateNode::on_safety(const agv_interfaces::msg::SafetyStatus& msg) {
  if (!msg.safety_ok && safety_ok_) {
    RCLCPP_WARN(get_logger(), "safety_ok dropped: %s", msg.reason.c_str());
  } else if (msg.safety_ok && !safety_ok_) {
    RCLCPP_INFO(get_logger(), "safety_ok restored");
  }
  safety_ok_ = msg.safety_ok;
  last_safety_msg_ = now();
}

void CmdVelGateNode::on_hardware_estop(const std_msgs::msg::Bool& msg) {
  if (msg.data && !hardware_estop_) {
    RCLCPP_ERROR(get_logger(), "hardware E-stop asserted");
  } else if (!msg.data && hardware_estop_) {
    RCLCPP_INFO(get_logger(), "hardware E-stop cleared");
  }
  hardware_estop_ = msg.data;
}

void CmdVelGateNode::on_safety_timeout() {
  if (last_safety_msg_.nanoseconds() == 0) {
    return;  // never received yet — startup
  }
  const double age_s = (now() - last_safety_msg_).seconds();
  if (age_s > safety_timeout_s_ && safety_ok_) {
    RCLCPP_ERROR(get_logger(),
                 "safety_status stale (%.2fs > %.2fs) — gating cmd_vel",
                 age_s, safety_timeout_s_);
    safety_ok_ = false;
  }
}

}  // namespace agv_safety
