/**
 * Fusion Monitor Node
 *
 * 1. Subscribes to global EKF output (nav_msgs/Odometry)
 * 2. Republishes as /agv/pose (PoseWithCovarianceStamped) at 10Hz — spec requirement
 * 3. Monitors EKF covariance for localization degradation
 * 4. Publishes diagnostic_msgs/DiagnosticArray with localization health
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cmath>

class FusionMonitorNode : public rclcpp::Node
{
public:
  FusionMonitorNode() : Node("fusion_monitor")
  {
    // Parameters
    this->declare_parameter("pose_rate_hz", 10.0);
    this->declare_parameter("covariance_warn_threshold", 0.5);
    this->declare_parameter("covariance_error_threshold", 2.0);
    this->declare_parameter("stale_timeout_s", 2.0);

    pose_rate_hz_ = this->get_parameter("pose_rate_hz").as_double();
    cov_warn_ = this->get_parameter("covariance_warn_threshold").as_double();
    cov_error_ = this->get_parameter("covariance_error_threshold").as_double();
    stale_timeout_ = this->get_parameter("stale_timeout_s").as_double();

    // Subscribers
    sub_global_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/global", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_global_odom(msg); });

    sub_local_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/local", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) { last_local_time_ = this->now(); });

    // Publishers
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", rclcpp::QoS(10));

    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    // Timer for pose publishing at configured rate
    auto period = std::chrono::duration<double>(1.0 / pose_rate_hz_);
    timer_ = this->create_wall_timer(period, [this]() { publish_pose(); });

    // Diagnostic timer at 1Hz
    diag_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_diagnostics(); });

    RCLCPP_INFO(this->get_logger(),
      "Fusion monitor: pose at %.0fHz, cov warn=%.2f error=%.2f",
      pose_rate_hz_, cov_warn_, cov_error_);
  }

private:
  void on_global_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_global_odom_ = *msg;
    last_global_time_ = this->now();
    has_global_ = true;

    // Extract max diagonal covariance for health monitoring
    max_pose_cov_ = 0.0;
    for (int i : {0, 7, 35}) {  // x, y, yaw diagonal
      max_pose_cov_ = std::max(max_pose_cov_, std::abs(msg->pose.covariance[i]));
    }
  }

  void publish_pose()
  {
    if (!has_global_) return;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = last_global_odom_.header;
    pose_msg.pose = last_global_odom_.pose;
    pub_pose_->publish(pose_msg);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->now();

    // Localization status
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fusion_monitor: Localization";
    status.hardware_id = "agv_sensor_fusion";

    auto now = this->now();
    double global_age = has_global_ ? (now - last_global_time_).seconds() : 999.0;
    double local_age = (now - last_local_time_).seconds();

    // Determine health level
    if (global_age > stale_timeout_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Global EKF output stale";
    } else if (local_age > stale_timeout_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Local EKF output stale";
    } else if (max_pose_cov_ > cov_error_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Localization covariance critically high";
    } else if (max_pose_cov_ > cov_warn_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Localization covariance elevated";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Localization healthy";
    }

    auto kv = [](const std::string& k, double v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = std::to_string(v);
      return kv;
    };

    status.values.push_back(kv("max_pose_covariance", max_pose_cov_));
    status.values.push_back(kv("global_ekf_age_s", global_age));
    status.values.push_back(kv("local_ekf_age_s", local_age));

    diag_array.status.push_back(status);
    pub_diag_->publish(diag_array);
  }

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // State
  nav_msgs::msg::Odometry last_global_odom_;
  rclcpp::Time last_global_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_time_{0, 0, RCL_ROS_TIME};
  bool has_global_{false};
  double max_pose_cov_{0.0};

  // Parameters
  double pose_rate_hz_;
  double cov_warn_;
  double cov_error_;
  double stale_timeout_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
