#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <agv_interfaces/msg/rail_start_point.hpp>
#include <agv_interfaces/srv/rail_approach.hpp>
#include <agv_interfaces/srv/list_rail_starts.hpp>

#include <opencv2/core.hpp>

#include <mutex>
#include <string>
#include <unordered_map>

namespace agv_rail_approach {

enum class State {
  IDLE,
  COARSE_APPROACH,
  TAG_ACQUISITION,
  FINE_SERVOING,
  SETTLED,
  ABORTED
};

struct RailStart {
  int id;
  double x, y, yaw;
  double tag_size;
};

class RailApproachNode : public rclcpp::Node {
public:
  RailApproachNode();

private:
  // State machine
  State state_{State::IDLE};
  int target_tag_id_{-1};
  double desired_offset_x_{0.3};
  double desired_offset_y_{0.0};
  int settle_count_{0};
  rclcpp::Time tag_last_seen_;
  rclcpp::Time acquisition_start_;

  // Camera intrinsics
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  bool camera_info_received_{false};

  // Registry
  std::unordered_map<int, RailStart> rail_starts_;

  // Parameters
  std::string registry_file_;
  double default_tag_size_;
  double coarse_standoff_;
  double default_offset_x_;
  double tolerance_xy_;
  double tolerance_yaw_;
  int settle_frames_;
  double kp_linear_;
  double kp_lateral_;
  double kp_yaw_;
  double max_fine_linear_;
  double max_fine_angular_;
  double tag_loss_timeout_;
  double tag_reacquire_timeout_;
  double acquisition_timeout_;
  std::string camera_frame_;
  std::string base_frame_;

  // ROS interfaces
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Service<agv_interfaces::srv::RailApproach>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<agv_interfaces::srv::ListRailStarts>::SharedPtr list_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  using NavAction = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  NavGoalHandle::SharedPtr nav_goal_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Callbacks
  void on_detection(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_execute(
    const std::shared_ptr<rmw_request_id_t> request_id,
    const agv_interfaces::srv::RailApproach::Request::SharedPtr req,
    agv_interfaces::srv::RailApproach::Response::SharedPtr resp);
  void on_abort(
    const std::shared_ptr<rmw_request_id_t>,
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  void on_list(
    const std::shared_ptr<rmw_request_id_t>,
    const agv_interfaces::srv::ListRailStarts::Request::SharedPtr,
    agv_interfaces::srv::ListRailStarts::Response::SharedPtr resp);
  void publish_status();

  // State machine transitions
  void start_coarse_approach(const RailStart& rail);
  void on_nav_result(const NavGoalHandle::WrappedResult& result);
  void process_fine_servoing(int tag_id, const std::vector<cv::Point2d>& corners,
                              double tag_size);
  void finish(bool success, const std::string& message,
              double err_x = 0.0, double err_y = 0.0, double err_yaw = 0.0);
  void stop_robot();

  // Registry
  void load_registry();

  // Helpers
  std::string state_name() const;
};

}  // namespace agv_rail_approach
