// agv_markers — C++17 AprilTag pose correction node
//
// Subscribes to apriltag_ros detections (apriltag_msgs format),
// estimates tag pose via solvePnP, computes robot pose in map frame
// using known marker positions, publishes correction for global EKF.
//
// Canonical marker system: AprilTag family tag36h11
// AprilTags are pose anchors and drift correctors, NOT the sole localization strategy.

#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_localization/srv/set_pose.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

struct MarkerPose {
  double x, y, z;
  double yaw;
};

class MarkerCorrectionNode : public rclcpp::Node {
public:
  MarkerCorrectionNode() : Node("marker_correction") {
    declare_parameter("markers_registry_file", "");
    declare_parameter("max_detection_range", 5.0);
    declare_parameter("covariance_xy", 0.01);
    declare_parameter("covariance_yaw", 0.03);
    // Must match physical tag size. Registry uses 0.2m (20cm) tags.
    declare_parameter("tag_size", 0.2);
    declare_parameter("relocalization_threshold", 2.0);  // meters — force reset above this
    declare_parameter("min_confidence", 50.0);            // decision_margin minimum for reloc

    max_range_ = get_parameter("max_detection_range").as_double();
    cov_xy_ = get_parameter("covariance_xy").as_double();
    cov_yaw_ = get_parameter("covariance_yaw").as_double();
    tag_size_ = get_parameter("tag_size").as_double();
    reloc_threshold_ = get_parameter("relocalization_threshold").as_double();
    min_confidence_ = get_parameter("min_confidence").as_double();

    auto registry_file = get_parameter("markers_registry_file").as_string();
    if (!registry_file.empty()) load_registry(registry_file);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("marker_pose", 10);
    detected_pub_ = create_publisher<std_msgs::msg::String>("marker_detected", 10);

    // Subscribers
    detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", 10,
      std::bind(&MarkerCorrectionNode::on_detection, this, std::placeholders::_1));

    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed/zed_node/left/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!has_caminfo_) {
          fx_ = msg->k[0]; fy_ = msg->k[4];
          cx_ = msg->k[2]; cy_ = msg->k[5];
          has_caminfo_ = true;
          RCLCPP_INFO(get_logger(), "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            fx_, fy_, cx_, cy_);
        }
      });

    // Subscribe to global EKF output for current pose (relocalization check)
    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/global", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        current_ekf_x_ = msg->pose.pose.position.x;
        current_ekf_y_ = msg->pose.pose.position.y;
        has_ekf_pose_ = true;
      });

    // Service client to force-relocalize EKF when drift is catastrophic
    set_pose_client_ = create_client<robot_localization::srv::SetPose>("set_pose");

    RCLCPP_INFO(get_logger(), "Marker correction: %zu markers, tag_size=%.3fm, reloc_threshold=%.1fm",
      registry_.size(), tag_size_, reloc_threshold_);
  }

private:
  void load_registry(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open registry: %s", path.c_str());
      return;
    }

    // Simple YAML parser for our specific format
    int current_id = -1;
    MarkerPose current{0, 0, 0, 0};
    std::string line;
    while (std::getline(in, line)) {
      // Trim
      size_t start = line.find_first_not_of(" \t-");
      if (start == std::string::npos || line[start] == '#') continue;
      std::string trimmed = line.substr(start);

      if (trimmed.substr(0, 3) == "id:") {
        if (current_id >= 0) registry_[current_id] = current;
        current_id = std::stoi(trimmed.substr(3));
        current = {0, 0, 0, 0};
      } else if (trimmed.substr(0, 2) == "x:") {
        current.x = std::stod(trimmed.substr(2));
      } else if (trimmed.substr(0, 2) == "y:") {
        current.y = std::stod(trimmed.substr(2));
      } else if (trimmed.substr(0, 2) == "z:") {
        current.z = std::stod(trimmed.substr(2));
      } else if (trimmed.substr(0, 4) == "yaw:") {
        current.yaw = std::stod(trimmed.substr(4));
      }
    }
    if (current_id >= 0) registry_[current_id] = current;

    RCLCPP_INFO(get_logger(), "Loaded %zu markers from %s", registry_.size(), path.c_str());
    for (auto& [id, m] : registry_) {
      RCLCPP_INFO(get_logger(), "  tag_%d: (%.2f, %.2f, %.2f) yaw=%.2f", id, m.x, m.y, m.z, m.yaw);
    }
  }

  void on_detection(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (!has_caminfo_) return;

    for (const auto& det : msg->detections) {
      auto it = registry_.find(det.id);
      if (it == registry_.end()) continue;

      // Estimate tag pose in camera frame using solvePnP
      // Tag corners in tag frame (centered, CCW from bottom-left)
      double half = tag_size_ / 2.0;
      std::vector<cv::Point3d> obj_pts = {
        {-half, -half, 0}, { half, -half, 0},
        { half,  half, 0}, {-half,  half, 0}
      };

      std::vector<cv::Point2d> img_pts;
      for (const auto& c : det.corners) {
        img_pts.emplace_back(c.x, c.y);
      }

      if (img_pts.size() != 4) continue;

      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
      cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

      cv::Vec3d rvec, tvec;
      if (!cv::solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs, rvec, tvec)) {
        continue;
      }

      // Distance check
      double range = cv::norm(tvec);
      if (range > max_range_ || range < 0.1) continue;

      // Get camera→base_link transform from TF
      geometry_msgs::msg::TransformStamped cam_to_base;
      try {
        cam_to_base = tf_buffer_->lookupTransform("base_link", "zed_left_camera_frame",
          rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.5));
      } catch (...) { continue; }

      // Compute robot pose in map frame:
      // T_map_robot = T_map_tag × inverse(T_camera_tag) × T_camera_base
      // Simplified for 2D: use tag's known (x,y,yaw) and detected offset
      const auto& marker = it->second;

      // Tag pose in camera frame: tvec = (x_right, y_down, z_forward) in camera optical
      // Convert to base_link: forward = tvec[2], left = -tvec[0]
      double tag_fwd = tvec[2];   // forward distance to tag
      double tag_left = -tvec[0]; // lateral offset

      // Derive robot heading from the AprilTag observation instead of odometry.
      // rvec from solvePnP gives tag orientation in camera frame. Combined with
      // the tag's known yaw in the map (marker.yaw), we get an independent heading
      // estimate that can actually correct heading drift.
      double robot_yaw = 0.0;
      bool yaw_from_tag = false;
      {
        cv::Mat R_ct;
        cv::Rodrigues(rvec, R_ct);
        // Extract yaw of the tag as seen in the camera frame
        // R_ct columns: tag X/Y/Z axes in camera coords (optical: X-right, Y-down, Z-forward)
        // Tag yaw in camera = atan2(R_ct[0][0], R_ct[2][0]) projected onto camera XZ plane
        double tag_yaw_in_camera = std::atan2(R_ct.at<double>(0, 0), R_ct.at<double>(2, 0));
        // Robot heading = tag's known map yaw - observed yaw in camera + PI
        // The +PI accounts for the tag facing toward the approaching robot
        robot_yaw = marker.yaw - tag_yaw_in_camera + M_PI;
        // Normalize to [-PI, PI]
        while (robot_yaw > M_PI) robot_yaw -= 2.0 * M_PI;
        while (robot_yaw < -M_PI) robot_yaw += 2.0 * M_PI;
        yaw_from_tag = true;
      }

      // Fallback: use odom-based heading if rvec extraction failed
      if (!yaw_from_tag) {
        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
          odom_to_base = tf_buffer_->lookupTransform("odom", "base_link",
            rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.5));
          robot_yaw = std::atan2(
            2.0 * (odom_to_base.transform.rotation.w * odom_to_base.transform.rotation.z),
            1.0 - 2.0 * odom_to_base.transform.rotation.z * odom_to_base.transform.rotation.z);
          RCLCPP_WARN(get_logger(), "Tag %d: rvec yaw extraction failed, falling back to odom heading", det.id);
        } catch (...) { continue; }
      }

      // Robot position = tag position - offset rotated by robot heading
      // (tag is at known map position, robot sees it at angle relative to heading)
      double cos_yaw = std::cos(robot_yaw);
      double sin_yaw = std::sin(robot_yaw);
      double robot_x = marker.x - (tag_fwd * cos_yaw - tag_left * sin_yaw);
      double robot_y = marker.y - (tag_fwd * sin_yaw + tag_left * cos_yaw);

      // Build correction message
      geometry_msgs::msg::PoseWithCovarianceStamped correction;
      correction.header.stamp = now();
      correction.header.frame_id = "map";
      correction.pose.pose.position.x = robot_x;
      correction.pose.pose.position.y = robot_y;

      double half_yaw = robot_yaw / 2.0;
      correction.pose.pose.orientation.z = std::sin(half_yaw);
      correction.pose.pose.orientation.w = std::cos(half_yaw);

      // Quadratic range model: uncertainty grows with distance squared
      // At ref_range (2.0m), factor = 2.0. At 5m, factor = 7.25
      double ref_range = 2.0;
      double range_factor = 1.0 + (range / ref_range) * (range / ref_range);
      correction.pose.covariance[0] = 0.02 * range_factor;
      correction.pose.covariance[7] = 0.02 * range_factor;
      correction.pose.covariance[35] = 0.05 * range_factor;

      // Check if relocalization needed (large drift detected)
      double drift = 0.0;
      if (has_ekf_pose_) {
        drift = std::sqrt(
          std::pow(robot_x - current_ekf_x_, 2) +
          std::pow(robot_y - current_ekf_y_, 2));
      }

      if (drift >= reloc_threshold_ && det.decision_margin >= min_confidence_) {
        // RELOCALIZATION: drift too large for normal EKF correction
        // Force-reset EKF pose via set_pose service
        if (set_pose_client_->service_is_ready()) {
          auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
          request->pose = correction;
          set_pose_client_->async_send_request(request);
          RCLCPP_WARN(get_logger(),
            "RELOCALIZATION: Tag %d, drift=%.1fm (threshold=%.1fm), "
            "resetting EKF to (%.2f, %.2f)",
            det.id, drift, reloc_threshold_, robot_x, robot_y);
        }
      } else {
        // Normal correction — let EKF fuse smoothly
        pose_pub_->publish(correction);
      }

      std_msgs::msg::String det_msg;
      det_msg.data = "tag_" + std::to_string(det.id);
      detected_pub_->publish(det_msg);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Tag %d at %.2fm → (%.2f, %.2f) drift=%.1fm %s",
        det.id, range, robot_x, robot_y, drift,
        drift >= reloc_threshold_ ? "RELOC" : "smooth");
    }
  }

  // Registry
  std::map<int, MarkerPose> registry_;
  double max_range_, cov_xy_, cov_yaw_, tag_size_;
  double reloc_threshold_, min_confidence_;

  // Camera intrinsics
  bool has_caminfo_{false};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};

  // Current EKF pose (for drift detection)
  bool has_ekf_pose_{false};
  double current_ekf_x_{0}, current_ekf_y_{0};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}
