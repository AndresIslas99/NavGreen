#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_odrive/can_socket.hpp"
#include "agv_odrive/odrive_protocol.hpp"

namespace agv_odrive {

class ODriveCANNode : public rclcpp::Node {
public:
  ODriveCANNode();
  ~ODriveCANNode() override;

private:
  // -- Parameters --
  std::string can_interface_;
  uint8_t left_axis_id_;
  uint8_t right_axis_id_;
  double wheel_radius_;
  double track_width_;
  int publish_rate_hz_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  int cmd_vel_timeout_ms_;
  bool invert_left_;
  bool invert_right_;
  double left_sign_;    // +1.0 or -1.0
  double right_sign_;
  double left_scale_;
  double right_scale_;
  float min_effective_vel_;
  float stiction_torque_ff_;
  float max_wheel_accel_;
  float zero_vel_epsilon_;
  double gear_ratio_;  // motor_turns / wheel_turns (see odrive_can_node.cpp for docs)
  double max_fet_temp_;
  double max_motor_temp_;
  double critical_temp_offset_;

  // -- CAN --
  std::unique_ptr<CANSocket> can_;
  bool init_can();
  void read_can_messages();
  void request_encoders();
  void send_velocity(uint8_t node_id, float vel_turns_per_s);
  void send_velocity_with_ff(uint8_t node_id, float vel_turns_per_s, float torque_ff);
  float apply_wheel_shaping(float target, float& prev_cmd, double scale);
  void send_axis_state(uint8_t node_id, AxisState state);
  void stop_motors();

  // -- Per-axis state --
  struct AxisData {
    float position = 0.0f;      // turns (cumulative)
    float velocity = 0.0f;      // turns/s
    float prev_position = 0.0f; // for delta computation
    uint8_t state = 0;
    uint32_t errors = 0;
    bool heartbeat_received = false;
    float prev_cmd = 0.0f;  // previous commanded velocity for accel limiting
    float fet_temp = 0.0f;      // FET temperature °C
    float motor_temp = 0.0f;    // Motor temperature °C
  };
  AxisData left_;
  AxisData right_;

  // -- Odometry state --
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_theta_ = 0.0;
  bool odom_initialized_ = false;
  rclcpp::Time last_odom_time_;

  void integrate_odometry();
  void publish_odometry();
  void publish_joint_states();

  // -- E-stop --
  bool e_stop_active_ = false;

  // -- cmd_vel timeout --
  rclcpp::Time last_cmd_vel_time_;

  // -- Bus state --
  double bus_voltage_ = 0.0;
  double bus_current_ = 0.0;
  int diag_counter_ = 0;

  // -- Drive debug state (for drive_debug topic) --
  double last_linear_cmd_   = 0.0;
  double last_angular_cmd_  = 0.0;
  float  last_left_target_  = 0.0f;
  float  last_right_target_ = 0.0f;
  bool   zero_cmd_active_   = false;

  // -- Temperature monitoring --
  void check_temperature(const AxisData& axis);
  std::string thermal_state_{"ok"};

  // -- CAN retry backoff --
  int can_retry_delay_ms_{100};
  rclcpp::Time last_can_retry_{0, 0, RCL_ROS_TIME};

  // -- Motor readiness --
  bool motors_armed() const;
  void publish_motor_state();
  void on_motor_enable(const std_msgs::msg::Bool& msg);

  // -- Publishers --
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_motor_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_drive_debug_;

  // -- Subscribers --
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motor_enable_;

  // -- Callbacks --
  void on_cmd_vel(const geometry_msgs::msg::Twist& msg);
  void on_e_stop(const std_msgs::msg::Bool& msg);

  // -- Timers --
  rclcpp::TimerBase::SharedPtr timer_main_;
  rclcpp::TimerBase::SharedPtr timer_encoder_;
  rclcpp::TimerBase::SharedPtr timer_motor_state_;
  rclcpp::TimerBase::SharedPtr timer_debug_;

  void main_loop();
  void encoder_request_loop();
  void publish_drive_debug();
};

}  // namespace agv_odrive
