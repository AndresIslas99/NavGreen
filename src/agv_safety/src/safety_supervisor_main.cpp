#include "rclcpp/rclcpp.hpp"

#include "agv_safety/safety_supervisor.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_safety::SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
