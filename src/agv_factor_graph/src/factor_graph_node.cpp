/**
 * Factor Graph Node — sliding-window iSAM2 sensor fusion.
 *
 * Architecture:
 *   Local odometry (ekf_local) → BetweenFactor between consecutive poses
 *   Visual odometry (cuVSLAM)  → BetweenFactor between consecutive poses
 *   AprilTag detection         → PriorFactor (absolute pose)
 *
 * The graph is optimized incrementally with iSAM2. Old poses are
 * marginalized when the window exceeds window_size to keep computation bounded.
 *
 * Why factor graph instead of EKF:
 *   - Loop closures (e.g., AprilTag) propagate corrections backward in time
 *   - No teleportation/jumps when absolute corrections arrive
 *   - Multi-hypothesis tracking inherent (handles ambiguity better)
 *   - Cleaner mathematics for multi-rate, multi-source sensor fusion
 */

#include "agv_factor_graph/factor_graph_node.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/linear/NoiseModel.h>

using gtsam::symbol_shorthand::X;

namespace agv_factor_graph {

FactorGraphNode::FactorGraphNode() : Node("factor_graph") {
  // Parameters
  window_size_ = static_cast<size_t>(declare_parameter("window_size", 200));
  local_odom_xy_sigma_  = declare_parameter("local_odom_xy_sigma",  0.05);
  local_odom_yaw_sigma_ = declare_parameter("local_odom_yaw_sigma", 0.03);
  visual_odom_xy_sigma_  = declare_parameter("visual_odom_xy_sigma",  0.02);
  visual_odom_yaw_sigma_ = declare_parameter("visual_odom_yaw_sigma", 0.01);
  marker_xy_sigma_  = declare_parameter("marker_xy_sigma",  0.05);
  marker_yaw_sigma_ = declare_parameter("marker_yaw_sigma", 0.05);
  prior_xy_sigma_  = declare_parameter("prior_xy_sigma",  1.0);
  prior_yaw_sigma_ = declare_parameter("prior_yaw_sigma", 1.0);
  map_frame_  = declare_parameter("map_frame",  std::string("map"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  publish_tf_ = declare_parameter("publish_tf", false);

  // Noise models — diagonal Gaussian noise on (x, y, yaw)
  local_odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(local_odom_xy_sigma_, local_odom_xy_sigma_, local_odom_yaw_sigma_));
  visual_odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(visual_odom_xy_sigma_, visual_odom_xy_sigma_, visual_odom_yaw_sigma_));
  marker_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(marker_xy_sigma_, marker_xy_sigma_, marker_yaw_sigma_));
  prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(prior_xy_sigma_, prior_xy_sigma_, prior_yaw_sigma_));

  // iSAM2 setup
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

  // ROS interfaces
  // Subscribe to ekf_global output (in map frame). During parallel validation
  // the factor graph re-optimizes the same trajectory with AprilTag corrections.
  // After cutover (publish_tf=true), this becomes the authoritative pose.
  local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/global", rclcpp::SensorDataQoS(),
    std::bind(&FactorGraphNode::on_local_odom, this, std::placeholders::_1));

  visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", rclcpp::SensorDataQoS(),
    std::bind(&FactorGraphNode::on_visual_odom, this, std::placeholders::_1));

  marker_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "marker_pose", 10,
    std::bind(&FactorGraphNode::on_marker_pose, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("factor_graph/odometry", 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(),
    "Factor graph node ready: window=%zu, publish_tf=%s",
    window_size_, publish_tf_ ? "true" : "false");
}

gtsam::Pose2 FactorGraphNode::pose2_from_msg(const geometry_msgs::msg::Pose& p) {
  // Extract yaw from quaternion (assuming 2D motion)
  const double yaw = std::atan2(
    2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
    1.0 - 2.0 * (p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z));
  return gtsam::Pose2(p.position.x, p.position.y, yaw);
}

void FactorGraphNode::on_local_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(graph_mutex_);

  const gtsam::Pose2 current = pose2_from_msg(msg->pose.pose);

  if (!initialized_) {
    // First observation: anchor with prior at origin (or current local pose)
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), current, prior_noise_));
    initial_estimate_.insert(X(0), current);
    isam_->update(graph_, initial_estimate_);
    graph_.resize(0);
    initial_estimate_.clear();
    active_indices_.push_back(0);
    pose_index_ = 0;
    last_local_odom_ = current;
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized factor graph at (%.3f, %.3f, %.3f)",
                current.x(), current.y(), current.theta());
    publish_estimate(msg->header.stamp);
    return;
  }

  // Compute delta from last local odometry
  const gtsam::Pose2 delta = last_local_odom_->between(current);
  last_local_odom_ = current;

  // Skip if delta is essentially zero (avoid degenerate factors)
  if (delta.translation().norm() < 1e-4 && std::abs(delta.theta()) < 1e-4) {
    return;
  }

  // Add new pose node connected to previous by BetweenFactor
  const size_t prev_idx = pose_index_;
  ++pose_index_;
  graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
    X(prev_idx), X(pose_index_), delta, local_odom_noise_));

  // Initial estimate: propagate from previous pose
  const gtsam::Pose2 prev_pose = isam_->calculateEstimate<gtsam::Pose2>(X(prev_idx));
  initial_estimate_.insert(X(pose_index_), prev_pose * delta);

  // Incremental update
  isam_->update(graph_, initial_estimate_);
  graph_.resize(0);
  initial_estimate_.clear();

  active_indices_.push_back(pose_index_);
  slide_window();

  publish_estimate(msg->header.stamp);
}

void FactorGraphNode::on_visual_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(graph_mutex_);

  if (!initialized_) return;  // wait for local odom to anchor

  const gtsam::Pose2 current = pose2_from_msg(msg->pose.pose);

  if (!last_visual_odom_) {
    last_visual_odom_ = current;
    return;
  }

  // Compute delta
  const gtsam::Pose2 delta = last_visual_odom_->between(current);
  last_visual_odom_ = current;

  if (delta.translation().norm() < 1e-4 && std::abs(delta.theta()) < 1e-4) {
    return;
  }

  // cuVSLAM provides differential measurements between consecutive frames.
  // We add it as an additional BetweenFactor on the most recent pose pair.
  // This is a slight approximation: visual delta is between visual frame N-1
  // and N, but we apply it to the last two graph nodes (which are local-odom-driven).
  // For tightly synchronized rates this is acceptable.
  if (pose_index_ < 1) return;
  graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
    X(pose_index_ - 1), X(pose_index_), delta, visual_odom_noise_));

  isam_->update(graph_);
  graph_.resize(0);
}

void FactorGraphNode::on_marker_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(graph_mutex_);

  if (!initialized_) return;

  const gtsam::Pose2 marker_pose = pose2_from_msg(msg->pose.pose);

  // Add absolute prior on the most recent pose.
  // This is the key advantage of factor graphs: this correction propagates
  // backward through the entire window via re-optimization.
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(pose_index_), marker_pose, marker_noise_));
  isam_->update(graph_);
  graph_.resize(0);

  RCLCPP_INFO(get_logger(),
    "Marker correction applied at pose %zu: (%.3f, %.3f, %.3f)",
    pose_index_, marker_pose.x(), marker_pose.y(), marker_pose.theta());

  // Republish corrected estimate
  publish_estimate(msg->header.stamp);
}

void FactorGraphNode::slide_window() {
  // Marginalize oldest pose if window exceeds limit
  while (active_indices_.size() > window_size_) {
    const size_t to_marginalize = active_indices_.front();
    active_indices_.pop_front();

    // GTSAM ISAM2::marginalizeLeaves expects FastList<Key>, not vector
    gtsam::FastList<gtsam::Key> to_remove;
    to_remove.push_back(X(to_marginalize));

    try {
      isam_->marginalizeLeaves(to_remove);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Failed to marginalize pose %zu: %s", to_marginalize, e.what());
    }
  }
}

void FactorGraphNode::publish_estimate(const rclcpp::Time& stamp) {
  if (!initialized_) return;

  gtsam::Pose2 latest;
  try {
    latest = isam_->calculateEstimate<gtsam::Pose2>(X(pose_index_));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to retrieve latest estimate: %s", e.what());
    return;
  }

  // Build odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_msg.pose.pose.position.x = latest.x();
  odom_msg.pose.pose.position.y = latest.y();
  odom_msg.pose.pose.position.z = 0.0;
  const double half_yaw = latest.theta() / 2.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(half_yaw);
  odom_msg.pose.pose.orientation.w = std::cos(half_yaw);
  odom_pub_->publish(odom_msg);

  // Optionally publish TF map -> odom (cutover mode)
  if (publish_tf_ && tf_broadcaster_) {
    // We have map -> base_link from the factor graph.
    // We need to publish map -> odom such that:
    //   map -> odom -> base_link == map -> base_link
    // i.e., map -> odom = (map -> base_link) * (base_link -> odom)
    //                   = (map -> base_link) * inverse(odom -> base_link)
    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
      odom_to_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
        rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.1));
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup odom->base_link failed: %s", e.what());
      return;
    }

    const double odom_yaw = std::atan2(
      2.0 * (odom_to_base.transform.rotation.w * odom_to_base.transform.rotation.z),
      1.0 - 2.0 * (odom_to_base.transform.rotation.z * odom_to_base.transform.rotation.z));
    const gtsam::Pose2 odom_base(
      odom_to_base.transform.translation.x,
      odom_to_base.transform.translation.y,
      odom_yaw);

    // map -> odom = (map -> base_link) * inverse(odom -> base_link)
    const gtsam::Pose2 map_odom = latest * odom_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = stamp;
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = map_odom.x();
    map_to_odom.transform.translation.y = map_odom.y();
    map_to_odom.transform.translation.z = 0.0;
    const double half_map_yaw = map_odom.theta() / 2.0;
    map_to_odom.transform.rotation.x = 0.0;
    map_to_odom.transform.rotation.y = 0.0;
    map_to_odom.transform.rotation.z = std::sin(half_map_yaw);
    map_to_odom.transform.rotation.w = std::cos(half_map_yaw);
    tf_broadcaster_->sendTransform(map_to_odom);
  }
}

}  // namespace agv_factor_graph

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_factor_graph::FactorGraphNode>());
  rclcpp::shutdown();
  return 0;
}
