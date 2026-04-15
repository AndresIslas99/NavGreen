#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "agv_interfaces/msg/safety_status.hpp"

namespace agv_safety {

// Tracks the freshness of a single monitored topic.
struct MonitoredTopic {
  std::string name;
  std::string type;            // ROS2 message type as a string, e.g. "nav_msgs/msg/Odometry"
  std::chrono::milliseconds deadline{0};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  bool ever_seen{false};
  rclcpp::GenericSubscription::SharedPtr sub;  // type-erased
};

// Pure logic helper exposed for unit testing without spinning a node.
struct SupervisorVerdict {
  bool safety_ok{true};
  std::vector<std::string> silent_topics;
  std::vector<std::string> alerts;
  std::string reason;
};

SupervisorVerdict evaluate_topics(
    const std::vector<MonitoredTopic>& topics,
    const rclcpp::Time& now,
    const rclcpp::Time& started_at,
    bool software_estop_latched,
    std::chrono::milliseconds startup_grace);

class SafetySupervisorNode : public rclcpp::Node {
 public:
  SafetySupervisorNode();

 private:
  void declare_parameters();
  void load_monitored_topics();
  void on_tick();
  void on_software_estop(const std_msgs::msg::Bool& msg);

  std::vector<MonitoredTopic> topics_;
  bool software_estop_latched_{false};
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  std::chrono::milliseconds startup_grace_{2000};
  double publish_rate_hz_{10.0};

  rclcpp::Publisher<agv_interfaces::msg::SafetyStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agv_safety
