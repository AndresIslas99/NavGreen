#include "agv_rail_approach/rail_approach_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_rail_approach::RailApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
