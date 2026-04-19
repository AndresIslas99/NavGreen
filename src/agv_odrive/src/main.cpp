#include "rclcpp/rclcpp.hpp"
#include "agv_odrive/odrive_can_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_odrive::ODriveCANNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
