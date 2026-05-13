#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "agv_interfaces/msg/safety_status.hpp"

namespace agv_safety {

// Pure logic helper exposed for unit testing.
struct GateInputs {
  geometry_msgs::msg::Twist input_cmd;
  bool safety_ok{true};
  bool hardware_estop{false};
  double max_linear{0.5};
  double max_angular{1.5};
};

geometry_msgs::msg::Twist apply_gate(const GateInputs& in);

class CmdVelGateNode : public rclcpp::Node {
 public:
  CmdVelGateNode();

 private:
  void declare_parameters();
  void on_input(const geometry_msgs::msg::Twist& msg);
  void on_safety(const agv_interfaces::msg::SafetyStatus& msg);
  void on_hardware_estop(const std_msgs::msg::Bool& msg);
  void on_safety_timeout();

  // Fail-CLOSED defaults: until the first SafetyStatus arrives, every
  // cmd_vel input is gated to zero. apply_gate() reads safety_ok_ on
  // every callback. The on_safety_timeout watchdog explicitly does
  // NOT fire while last_safety_msg_ is unset (gate.cpp:95) — so the
  // startup window is "always zero" rather than "always zero after a
  // short grace". Verified 2026-05-13 (Sprint D, HIGH-09-04).
  // Distinction: GateInputs::safety_ok defaults to true (permissive)
  // because it is only used by the pure-logic unit tests where the
  // tester explicitly sets safety_ok=false to exercise the
  // zero-output branch. The DRIVING member is the one below.
  bool safety_ok_{false};
  bool hardware_estop_{false};
  rclcpp::Time last_safety_msg_{0, 0, RCL_ROS_TIME};
  double safety_timeout_s_{0.5};
  double max_linear_{0.5};
  double max_angular_{1.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_input_;
  rclcpp::Subscription<agv_interfaces::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hw_estop_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_output_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace agv_safety
