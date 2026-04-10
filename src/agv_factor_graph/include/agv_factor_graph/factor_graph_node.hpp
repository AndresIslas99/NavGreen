#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace agv_factor_graph {

/// Sliding-window iSAM2-based pose estimator.
///
/// Replaces ekf_global. Maintains a sliding window of the last N poses and
/// re-optimizes incrementally as new sensor data arrives. Unlike an EKF, when
/// an absolute correction (e.g., from an AprilTag) arrives, the optimization
/// propagates the correction backward through the window, eliminating the
/// position "jumps" that EKF-based systems exhibit.
///
/// Inputs:
///   - odometry/local      : nav_msgs/Odometry from ekf_local (continuous)
///   - /visual_slam/tracking/odometry : nav_msgs/Odometry from cuVSLAM
///   - marker_pose         : geometry_msgs/PoseWithCovarianceStamped from AprilTags
///
/// Outputs:
///   - factor_graph/odometry : nav_msgs/Odometry (the smoothed estimate)
///   - TF map -> odom (only when publish_tf:=true)
class FactorGraphNode : public rclcpp::Node {
public:
  FactorGraphNode();

private:
  // Callbacks
  void on_local_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_marker_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // Helpers
  static gtsam::Pose2 pose2_from_msg(const geometry_msgs::msg::Pose& p);
  void publish_estimate(const rclcpp::Time& stamp);
  void slide_window();

  // Parameters
  size_t window_size_;
  double local_odom_xy_sigma_;
  double local_odom_yaw_sigma_;
  double visual_odom_xy_sigma_;
  double visual_odom_yaw_sigma_;
  double marker_xy_sigma_;
  double marker_yaw_sigma_;
  double prior_xy_sigma_;
  double prior_yaw_sigma_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;

  // GTSAM state (protected by mutex)
  std::mutex graph_mutex_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  std::deque<size_t> active_indices_;
  size_t pose_index_{0};
  bool initialized_{false};

  // Last observations for delta computation
  std::optional<gtsam::Pose2> last_local_odom_;
  std::optional<gtsam::Pose2> last_visual_odom_;

  // Noise models (built once at construction)
  gtsam::SharedNoiseModel local_odom_noise_;
  gtsam::SharedNoiseModel visual_odom_noise_;
  gtsam::SharedNoiseModel marker_noise_;
  gtsam::SharedNoiseModel prior_noise_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr marker_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace agv_factor_graph
