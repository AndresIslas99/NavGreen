#include "rclcpp/rclcpp.hpp"

#include "agv_safety/cmd_vel_gate.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_safety::CmdVelGateNode>());
  rclcpp::shutdown();
  return 0;
}
