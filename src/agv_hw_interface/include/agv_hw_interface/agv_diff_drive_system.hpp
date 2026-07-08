#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "agv_hw_interface/can_socket.hpp"

namespace agv_hw_interface {

// ros2_control SystemInterface plugin for the AGV ODrive CAN driver.
//
// Exposes two joints (left_wheel_joint, right_wheel_joint), each with:
//   state interfaces:    position (rad), velocity (rad/s)
//   command interfaces:  velocity (rad/s)
//
// Hardware parameters (read from <ros2_control><hardware><param> in URDF):
//   can_interface           — SocketCAN interface name (default "can0")
//   left_axis_id            — ODrive CAN node id for left motor (default 0)
//   right_axis_id           — ODrive CAN node id for right motor (default 1)
//   gear_ratio              — motor turns / wheel turns (default 10.0)
//   invert_left             — bool, default true
//   invert_right            — bool, default false
//   recv_timeout_ms         — int, poll timeout per read() (default 5)
//   max_send_failures       — consecutive write() send failures before the
//                             plugin reports ERROR (default 10)
//   encoder_timeout_cycles  — consecutive read() cycles without any encoder
//                             frame before the plugin reports ERROR
//                             (default 100 ≈ 2 s at the 50 Hz update rate)
//
// On platforms without can0, the plugin still loads but stays in the
// inactive state. mock_components/GenericSystem is the recommended option for
// development without hardware (see launch/agv_ros2control_mock.launch.py).
class AgvDiffDriveSystem : public hardware_interface::SystemInterface {
 public:
  AgvDiffDriveSystem() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Hardware parameters
  std::string can_interface_{"can0"};
  uint8_t left_axis_id_{0};
  uint8_t right_axis_id_{1};
  double gear_ratio_{10.0};
  bool invert_left_{true};
  bool invert_right_{false};
  int recv_timeout_ms_{5};
  int max_send_failures_{10};
  int encoder_timeout_cycles_{100};

  // Fault counters (reset on every successful cycle / on_activate)
  int send_fail_count_{0};
  int stale_read_cycles_{0};

  // CAN socket (null until on_activate)
  std::unique_ptr<CANSocket> can_;

  // Per-joint state (radians and rad/s as ros2_control expects)
  struct WheelState {
    double position{0.0};        // rad (cumulative)
    double velocity{0.0};        // rad/s
    double command{0.0};         // rad/s (from controller)
    double last_motor_turns{0.0};  // last raw motor position (turns) for delta
    bool initialized{false};
  };
  WheelState left_;
  WheelState right_;

  rclcpp::Logger logger_ = rclcpp::get_logger("agv_hw_interface");

  bool send_velocity(uint8_t node_id, double wheel_rad_per_s, bool invert);
  void request_encoders(uint8_t node_id);
  void process_encoder_frame(uint8_t node_id, const struct can_frame& frame);
};

}  // namespace agv_hw_interface
