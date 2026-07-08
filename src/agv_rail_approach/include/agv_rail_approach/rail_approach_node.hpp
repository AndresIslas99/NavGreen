#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
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

#include "agv_rail_approach/fine_servo_controller.hpp"

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
  std::string runtime_registry_file_;
  double default_tag_size_;
  double coarse_standoff_;
  double default_offset_x_;
  double tolerance_xy_;
  double tolerance_yaw_;
  int settle_frames_;
  double kp_linear_;
  double kp_lateral_;
  double kp_yaw_;
  // Iter-46 Paso 1.b: PI + stiction feedforward on the forward axis.
  // See fine_servo_controller.hpp FineServoState / fine_servo_compute
  // for the control law. ki_linear_ defaults to 0 (pure-P legacy).
  // stiction_ff_vel_mps_ defaults to 0 (no FF). HIL launch override
  // sets both; real-robot yaml keeps them zero until ODrive char
  // confirms the deadband.
  double ki_linear_{0.0};
  double stiction_ff_vel_mps_{0.0};
  FineServoState fine_servo_state_;
  rclcpp::Time last_fine_servo_tick_{0, 0, RCL_ROS_TIME};
  double max_fine_linear_;
  double max_fine_angular_;
  bool check_yaw_convergence_{false};
  // Iter-11 / Option B: observability for fine-servo rejections.
  // last_reject_reason_ is "none" by default and updated to
  // verdict_to_str(...) when fine_servo_step returns a non-OK verdict
  // OR when the TF lookup fails. Both get stamped; publish_status
  // includes the age so stale rejects decay without needing a clear.
  std::string last_reject_reason_{"none"};
  rclcpp::Time last_reject_stamp_{0, 0, RCL_ROS_TIME};
  // Iter-12 / Option C: smooth solvePnP jitter before the controller.
  // Iter-42: window 5 → 15. tools/solvepnp_noise_benchmark.py (1000
  // Monte-Carlo with ±1 px corner noise, ZED 2i VGA NATIVE, tag at 1.77 m)
  // shows σ(tvec.z) drops 26 mm (raw) → 14 mm (median5) → 8 mm (median15).
  // Lag cost 0.75 s at 20 Hz is well inside max_fine_duration_s=240 s.
  int pnp_filter_window_{15};
  TvecRvecMedianFilter pnp_filter_{15};
  double tag_loss_timeout_;
  double tag_reacquire_timeout_;
  double acquisition_timeout_;
  // Iter-13 / Option D: max wall-time allowed in FINE_SERVOING.
  // Zero or negative disables the check (the harness-level deadline
  // still applies).
  double max_fine_duration_{120.0};
  rclcpp::Time fine_servo_start_{0, 0, RCL_ROS_TIME};
  // Max wall-time allowed in COARSE_APPROACH. Covers the case where the
  // Nav2 goal response/result never arrives (server died mid-goal) —
  // without a deadline the node would stay in COARSE_APPROACH forever.
  // Zero or negative disables the check.
  double coarse_timeout_{180.0};
  rclcpp::Time coarse_start_{0, 0, RCL_ROS_TIME};
  // Iter-15: if robot is already within this radius of the target tag
  // (measured against map→base_link), skip Nav2 coarse_approach and
  // jump straight to TAG_ACQUISITION. Default 2.0 m covers every Round-44
  // teleport spawn distance. Set to 0 to always run Nav2 coarse.
  double coarse_skip_radius_{2.0};
  std::string camera_frame_;
  std::string base_frame_;
  // Iter-44 Fase 2 Arch A (Registry-aware longitudinal):
  //   solvePnP on grazing floor tags is weakly conditioned in the forward
  //   (tvec.z) component — σ≈26 mm raw, 8 mm post-median15, WITH a bias
  //   that scales with incidence angle. Meanwhile the registry gives the
  //   tag's map pose to sub-mm, and TF map→cam_optical is precise whenever
  //   the EKF is synced (gt_to_wheel_odom in HIL, marker_correction RELOC
  //   in production). When `use_registry_longitudinal` is true, we override
  //   tvec_used[2] (forward) with the TF+registry estimate while keeping
  //   PnP for tvec_used[0] (lateral, well-conditioned) and rvec (yaw).
  //   This cuts the worst-DOF noise without touching the dimensions where
  //   PnP is already sub-cm. Default false to preserve backwards-compat
  //   with production launches until A/B validates the HIL gain.
  bool use_registry_longitudinal_{false};
  double registry_max_stale_s_{2.0};  // reject registry override if TF older

  // ROS interfaces
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Service<agv_interfaces::srv::RailApproach>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<agv_interfaces::srv::ListRailStarts>::SharedPtr list_srv_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reload_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loc_state_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  // Localization state cached for the gate in on_execute. The gate is
  // active only when skip_coarse_approach=false: coarse_approach uses
  // map→base_link TF and Nav2, both unsafe without a verified anchor.
  // Skip-coarse paths use direct AprilTag detection and don't need
  // map; we accept any localization state.
  std::string last_localization_action_{"UNKNOWN"};

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
  void on_status_timer();
  void check_deadlines();
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
  void reload_all_registries();
  void load_registry_from(const std::string& path);

  // Helpers
  std::string state_name() const;
  std::string last_reject_age_str() const;
};

}  // namespace agv_rail_approach
