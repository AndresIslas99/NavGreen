#include "agv_rail_approach/rail_approach_node.hpp"

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace agv_rail_approach {

RailApproachNode::RailApproachNode() : Node("rail_approach") {
  // Parameters
  declare_parameter("registry_file", "");
  declare_parameter("runtime_registry_file", "");
  declare_parameter("tag_size", 0.2);
  declare_parameter("coarse_standoff_distance", 0.5);
  declare_parameter("default_offset_x", 0.3);
  declare_parameter("default_offset_y", 0.0);
  declare_parameter("tolerance_xy", 0.003);
  declare_parameter("tolerance_yaw", 0.015);
  declare_parameter("settle_frames", 5);
  declare_parameter("Kp_linear", 0.15);
  declare_parameter("Kp_lateral", 0.3);
  declare_parameter("Kp_yaw", 0.5);
  declare_parameter("max_fine_linear_vel", 0.03);
  declare_parameter("max_fine_angular_vel", 0.10);
  declare_parameter("tag_loss_timeout_s", 0.5);
  declare_parameter("tag_reacquire_timeout_s", 3.0);
  declare_parameter("acquisition_timeout_s", 3.0);
  declare_parameter("camera_frame", "zed_left_camera_optical_frame");
  declare_parameter("base_frame", "base_link");
  declare_parameter("camera_info_topic", "/agv/zed/left/camera_info");
  declare_parameter("detections_topic", "detections");

  registry_file_ = get_parameter("registry_file").as_string();
  runtime_registry_file_ = get_parameter("runtime_registry_file").as_string();
  default_tag_size_ = get_parameter("tag_size").as_double();
  coarse_standoff_ = get_parameter("coarse_standoff_distance").as_double();
  default_offset_x_ = get_parameter("default_offset_x").as_double();
  tolerance_xy_ = get_parameter("tolerance_xy").as_double();
  tolerance_yaw_ = get_parameter("tolerance_yaw").as_double();
  settle_frames_ = get_parameter("settle_frames").as_int();
  kp_linear_ = get_parameter("Kp_linear").as_double();
  kp_lateral_ = get_parameter("Kp_lateral").as_double();
  kp_yaw_ = get_parameter("Kp_yaw").as_double();
  max_fine_linear_ = get_parameter("max_fine_linear_vel").as_double();
  max_fine_angular_ = get_parameter("max_fine_angular_vel").as_double();
  tag_loss_timeout_ = get_parameter("tag_loss_timeout_s").as_double();
  tag_reacquire_timeout_ = get_parameter("tag_reacquire_timeout_s").as_double();
  acquisition_timeout_ = get_parameter("acquisition_timeout_s").as_double();
  camera_frame_ = get_parameter("camera_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  auto cam_info_topic = get_parameter("camera_info_topic").as_string();
  auto detections_topic = get_parameter("detections_topic").as_string();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    detections_topic, 10,
    std::bind(&RailApproachNode::on_detection, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cam_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&RailApproachNode::on_camera_info, this, std::placeholders::_1));

  // Publishers
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("rail_approach/status", 10);
  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("rail_approach/target_pose", 10);

  // Services
  execute_srv_ = create_service<agv_interfaces::srv::RailApproach>(
    "rail_approach/execute",
    std::bind(&RailApproachNode::on_execute, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  abort_srv_ = create_service<std_srvs::srv::Trigger>(
    "rail_approach/abort",
    std::bind(&RailApproachNode::on_abort, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  list_srv_ = create_service<agv_interfaces::srv::ListRailStarts>(
    "rail_approach/list_rail_starts",
    std::bind(&RailApproachNode::on_list, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");

  // Status timer (2 Hz)
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RailApproachNode::publish_status, this));

  // Hot reload trigger from ui_backend (when operator assigns/removes tags)
  auto reload_qos = rclcpp::QoS(1).transient_local().reliable();
  reload_sub_ = create_subscription<std_msgs::msg::Empty>(
    "markers/registry_reload", reload_qos,
    [this](std_msgs::msg::Empty::SharedPtr) {
      RCLCPP_INFO(get_logger(), "Reloading rail starts from disk");
      reload_all_registries();
    });

  reload_all_registries();

  RCLCPP_INFO(get_logger(), "Rail approach node ready, %zu rail starts loaded", rail_starts_.size());
}

// Reload from both static (registry_file) and runtime (runtime_registry_file) sources
void RailApproachNode::reload_all_registries() {
  rail_starts_.clear();
  if (!registry_file_.empty()) load_registry_from(registry_file_);
  if (!runtime_registry_file_.empty()) {
    std::ifstream test(runtime_registry_file_);
    if (test.is_open()) {
      test.close();
      load_registry_from(runtime_registry_file_);
    }
  }
}

// Original load_registry() now wraps load_registry_from for backward compatibility
void RailApproachNode::load_registry() {
  if (!registry_file_.empty()) load_registry_from(registry_file_);
}

// ── Registry loading ──

void RailApproachNode::load_registry_from(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_WARN(get_logger(), "Cannot open registry: %s", path.c_str());
    return;
  }

  // Simple YAML parser: find entries with type: rail_start
  // Reads line-by-line looking for marker blocks with type field
  std::string line;
  int current_id = -1;
  double x = 0, y = 0, yaw = 0;
  double tag_size = default_tag_size_;
  bool is_rail_start = false;
  bool in_marker = false;

  auto flush_marker = [&]() {
    if (in_marker && is_rail_start && current_id >= 0) {
      rail_starts_[current_id] = {current_id, x, y, yaw, tag_size};
      RCLCPP_INFO(get_logger(), "Rail start: tag %d at (%.2f, %.2f) yaw=%.2f size=%.3f",
                  current_id, x, y, yaw, tag_size);
    }
    current_id = -1;
    x = y = yaw = 0;
    tag_size = default_tag_size_;
    is_rail_start = false;
  };

  while (std::getline(file, line)) {
    // Skip comments and empty lines
    auto trimmed = line;
    auto pos = trimmed.find_first_not_of(' ');
    if (pos == std::string::npos || trimmed[pos] == '#') continue;
    trimmed = trimmed.substr(pos);

    // Detect new marker entry
    if (trimmed.find("- id:") == 0) {
      flush_marker();
      in_marker = true;
      try { current_id = std::stoi(trimmed.substr(5)); } catch (...) { current_id = -1; }
      continue;
    }

    if (!in_marker) continue;

    // Parse fields
    auto parse_double = [](const std::string& s, const std::string& key) -> double {
      auto p = s.find(key);
      if (p == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
      try { return std::stod(s.substr(p + key.size())); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
    };

    auto val = parse_double(trimmed, "x:");
    if (!std::isnan(val)) { x = val; continue; }
    val = parse_double(trimmed, "y:");
    if (!std::isnan(val)) { y = val; continue; }
    val = parse_double(trimmed, "yaw:");
    if (!std::isnan(val)) { yaw = val; continue; }
    val = parse_double(trimmed, "tag_size:");
    if (!std::isnan(val)) { tag_size = val; continue; }

    if (trimmed.find("type:") != std::string::npos) {
      if (trimmed.find("rail_start") != std::string::npos) {
        is_rail_start = true;
      }
    }
  }
  flush_marker();  // last entry
}

// ── Camera info ──

void RailApproachNode::on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (camera_info_received_) return;
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  if (fx_ > 0 && fy_ > 0) {
    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(), "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
  }
}

// ── Service: execute ──

void RailApproachNode::on_execute(
    const std::shared_ptr<rmw_request_id_t>,
    const agv_interfaces::srv::RailApproach::Request::SharedPtr req,
    agv_interfaces::srv::RailApproach::Response::SharedPtr resp) {

  if (state_ != State::IDLE) {
    resp->success = false;
    resp->message = "Approach already in progress (state: " + state_name() + ")";
    return;
  }

  auto it = rail_starts_.find(req->tag_id);
  if (it == rail_starts_.end()) {
    resp->success = false;
    resp->message = "Unknown rail start tag ID: " + std::to_string(req->tag_id);
    return;
  }

  if (!camera_info_received_) {
    resp->success = false;
    resp->message = "Camera intrinsics not yet received";
    return;
  }

  target_tag_id_ = req->tag_id;
  desired_offset_x_ = (req->offset_x > 0.01) ? req->offset_x : default_offset_x_;
  desired_offset_y_ = req->offset_y;
  settle_count_ = 0;

  RCLCPP_INFO(get_logger(), "Starting rail approach to tag %d (offset: %.3f, %.3f)",
              target_tag_id_, desired_offset_x_, desired_offset_y_);

  start_coarse_approach(it->second);

  // Don't respond yet — response will be sent when approach completes.
  // For now, respond immediately with "in progress" since ROS2 services are synchronous.
  resp->success = true;
  resp->message = "Approach started";
}

// ── Service: abort ──

void RailApproachNode::on_abort(
    const std::shared_ptr<rmw_request_id_t>,
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr resp) {

  if (state_ == State::IDLE) {
    resp->success = true;
    resp->message = "Already idle";
    return;
  }

  RCLCPP_WARN(get_logger(), "Abort requested in state %s", state_name().c_str());
  finish(false, "Aborted by operator");
  resp->success = true;
  resp->message = "Approach aborted";
}

// ── Service: list rail starts ──

void RailApproachNode::on_list(
    const std::shared_ptr<rmw_request_id_t>,
    const agv_interfaces::srv::ListRailStarts::Request::SharedPtr,
    agv_interfaces::srv::ListRailStarts::Response::SharedPtr resp) {

  for (const auto& [id, rs] : rail_starts_) {
    agv_interfaces::msg::RailStartPoint pt;
    pt.tag_id = rs.id;
    pt.x = rs.x;
    pt.y = rs.y;
    pt.approach_yaw = rs.yaw;
    pt.tag_size = rs.tag_size;
    resp->rail_starts.push_back(pt);
  }
}

// ── State machine: coarse approach via Nav2 ──

void RailApproachNode::start_coarse_approach(const RailStart& rail) {
  state_ = State::COARSE_APPROACH;

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    finish(false, "Nav2 not available");
    return;
  }

  // Compute standoff pose: rail position minus standoff along approach heading
  auto goal_msg = NavAction::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position.x = rail.x - coarse_standoff_ * std::cos(rail.yaw);
  goal_msg.pose.pose.position.y = rail.y - coarse_standoff_ * std::sin(rail.yaw);
  double half_yaw = rail.yaw / 2.0;
  goal_msg.pose.pose.orientation.z = std::sin(half_yaw);
  goal_msg.pose.pose.orientation.w = std::cos(half_yaw);

  RCLCPP_INFO(get_logger(), "Nav2 goal: (%.2f, %.2f) yaw=%.2f",
              goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, rail.yaw);

  auto send_opts = rclcpp_action::Client<NavAction>::SendGoalOptions();
  send_opts.result_callback = std::bind(&RailApproachNode::on_nav_result, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_opts);
}

void RailApproachNode::on_nav_result(const NavGoalHandle::WrappedResult& result) {
  if (state_ != State::COARSE_APPROACH) return;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Coarse approach complete, acquiring tag %d", target_tag_id_);
    state_ = State::TAG_ACQUISITION;
    acquisition_start_ = now();
  } else {
    RCLCPP_ERROR(get_logger(), "Nav2 coarse approach failed (code=%d)", static_cast<int>(result.code));
    finish(false, "Nav2 coarse approach failed");
  }
}

// ── Detection callback: drives TAG_ACQUISITION and FINE_SERVOING ──

void RailApproachNode::on_detection(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {

  if (state_ != State::TAG_ACQUISITION && state_ != State::FINE_SERVOING) return;
  if (!camera_info_received_) return;

  // Find target tag in detections
  for (const auto& det : msg->detections) {
    if (det.id != target_tag_id_) continue;

    // Extract pixel corners
    std::vector<cv::Point2d> corners;
    for (const auto& c : det.corners) {
      corners.emplace_back(c.x, c.y);
    }
    if (corners.size() != 4) continue;

    // Get tag size for this rail start
    double tag_size = default_tag_size_;
    auto it = rail_starts_.find(target_tag_id_);
    if (it != rail_starts_.end()) {
      tag_size = it->second.tag_size;
    }

    if (state_ == State::TAG_ACQUISITION) {
      RCLCPP_INFO(get_logger(), "Tag %d acquired, starting fine servoing", target_tag_id_);
      state_ = State::FINE_SERVOING;
      tag_last_seen_ = now();
    }

    process_fine_servoing(det.id, corners, tag_size);
    return;
  }

  // Target tag not found in this frame
  if (state_ == State::TAG_ACQUISITION) {
    double elapsed = (now() - acquisition_start_).seconds();
    if (elapsed > acquisition_timeout_) {
      finish(false, "Tag " + std::to_string(target_tag_id_) + " not found within timeout");
    }
  } else if (state_ == State::FINE_SERVOING) {
    double since_last = (now() - tag_last_seen_).seconds();
    if (since_last > tag_loss_timeout_) {
      stop_robot();
      if (since_last > tag_reacquire_timeout_) {
        finish(false, "Tag lost during fine servoing");
      }
    }
  }
}

// ── Fine servoing: visual feedback loop ──

void RailApproachNode::process_fine_servoing(
    int /*tag_id*/, const std::vector<cv::Point2d>& corners, double tag_size) {

  tag_last_seen_ = now();

  // solvePnP: tag corners → tag pose in camera optical frame
  double half = tag_size / 2.0;
  std::vector<cv::Point3d> obj_pts = {
    {-half, -half, 0}, { half, -half, 0},
    { half,  half, 0}, {-half,  half, 0}
  };

  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(obj_pts, corners, camera_matrix, dist_coeffs, rvec, tvec)) {
    return;
  }

  double range = cv::norm(tvec);
  if (range < 0.05 || range > 5.0) return;

  // Transform tag position from camera optical frame to base_link
  geometry_msgs::msg::TransformStamped cam_to_base;
  try {
    cam_to_base = tf_buffer_->lookupTransform(base_frame_, camera_frame_,
      rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.5));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF lookup %s->%s failed: %s", camera_frame_.c_str(), base_frame_.c_str(), e.what());
    return;
  }

  tf2::Quaternion cam_to_base_q(
    cam_to_base.transform.rotation.x, cam_to_base.transform.rotation.y,
    cam_to_base.transform.rotation.z, cam_to_base.transform.rotation.w);
  tf2::Transform cam_to_base_tf(cam_to_base_q, tf2::Vector3(
    cam_to_base.transform.translation.x, cam_to_base.transform.translation.y,
    cam_to_base.transform.translation.z));

  // tvec is in camera optical frame (X-right, Y-down, Z-forward)
  tf2::Vector3 tag_in_cam(tvec[0], tvec[1], tvec[2]);
  tf2::Vector3 tag_in_base = cam_to_base_tf * tag_in_cam;

  // Extract tag yaw in base_link frame from rvec
  cv::Mat R_ct;
  cv::Rodrigues(rvec, R_ct);
  double tag_yaw_in_cam = std::atan2(R_ct.at<double>(0, 0), R_ct.at<double>(2, 0));

  // Compute errors in base_link frame
  // Forward error: how far off from desired offset
  double error_x = tag_in_base.x() - desired_offset_x_;
  // Lateral error: tag should be centered (or at desired_offset_y)
  double error_y = tag_in_base.y() - desired_offset_y_;
  // Yaw error: tag_yaw_in_cam should be ~PI (tag faces robot)
  // Normalize to [-PI, PI]
  double error_yaw = tag_yaw_in_cam - M_PI;
  while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;

  // Publish target pose for visualization
  geometry_msgs::msg::PoseStamped target_msg;
  target_msg.header.stamp = now();
  target_msg.header.frame_id = base_frame_;
  target_msg.pose.position.x = tag_in_base.x();
  target_msg.pose.position.y = tag_in_base.y();
  target_msg.pose.position.z = tag_in_base.z();
  target_pose_pub_->publish(target_msg);

  // P-controller
  double cmd_linear = std::clamp(kp_linear_ * error_x, -max_fine_linear_, max_fine_linear_);
  double cmd_angular = std::clamp(
    kp_yaw_ * error_yaw + kp_lateral_ * error_y,
    -max_fine_angular_, max_fine_angular_);

  // Check convergence
  if (std::abs(error_x) < tolerance_xy_ &&
      std::abs(error_y) < tolerance_xy_ &&
      std::abs(error_yaw) < tolerance_yaw_) {
    settle_count_++;
    if (settle_count_ >= settle_frames_) {
      stop_robot();
      RCLCPP_INFO(get_logger(), "Settled! error: x=%.4f y=%.4f yaw=%.4f",
                  error_x, error_y, error_yaw);
      finish(true, "Precision approach complete", error_x, error_y, error_yaw);
      return;
    }
  } else {
    settle_count_ = 0;
  }

  // Publish velocity command
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = cmd_linear;
  cmd.angular.z = cmd_angular;
  cmd_pub_->publish(cmd);
}

// ── Finish (success or failure) ──

void RailApproachNode::finish(bool success, const std::string& message,
                               double err_x, double err_y, double err_yaw) {
  stop_robot();

  if (state_ == State::COARSE_APPROACH && nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_ = nullptr;
  }

  state_ = State::IDLE;
  target_tag_id_ = -1;
  settle_count_ = 0;

  if (success) {
    RCLCPP_INFO(get_logger(), "Approach complete: %s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Approach failed: %s", message.c_str());
  }

  (void)err_x; (void)err_y; (void)err_yaw;
}

void RailApproachNode::stop_robot() {
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

// ── Status publishing ──

void RailApproachNode::publish_status() {
  std_msgs::msg::String msg;
  msg.data = "{\"state\":\"" + state_name() + "\",\"target_tag\":" +
             std::to_string(target_tag_id_) + "}";
  status_pub_->publish(msg);
}

std::string RailApproachNode::state_name() const {
  switch (state_) {
    case State::IDLE: return "idle";
    case State::COARSE_APPROACH: return "coarse_approach";
    case State::TAG_ACQUISITION: return "tag_acquisition";
    case State::FINE_SERVOING: return "fine_servoing";
    case State::SETTLED: return "settled";
    case State::ABORTED: return "aborted";
  }
  return "unknown";
}

}  // namespace agv_rail_approach
