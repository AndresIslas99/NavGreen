/**
 * Covariance Override Node
 *
 * Relays Odometry and IMU messages, replacing zero covariances with
 * realistic values. Solves the Gazebo bridge zero-covariance problem
 * where the EKF treats sim data as infinitely precise.
 *
 * Subscribes: wheel_odom_raw, imu_raw, visual_odom_raw
 * Publishes:  wheel_odom, imu/data, visual_odom (with covariance filled)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

class CovarianceOverrideNode : public rclcpp::Node
{
public:
  CovarianceOverrideNode() : Node("covariance_override")
  {
    // Parameters
    declare_parameter("odom_pos_cov_xy", 0.001);
    declare_parameter("odom_pos_cov_yaw", 0.01);
    declare_parameter("odom_twist_cov_xy", 0.001);
    declare_parameter("odom_twist_cov_yaw", 0.01);
    declare_parameter("vslam_pos_cov_xy", 0.01);
    declare_parameter("vslam_pos_cov_yaw", 0.05);
    declare_parameter("imu_orient_cov", 0.001);
    declare_parameter("imu_gyro_cov", 0.0005);
    declare_parameter("imu_accel_cov", 0.01);

    odom_xy_ = get_parameter("odom_pos_cov_xy").as_double();
    odom_yaw_ = get_parameter("odom_pos_cov_yaw").as_double();
    odom_tw_xy_ = get_parameter("odom_twist_cov_xy").as_double();
    odom_tw_yaw_ = get_parameter("odom_twist_cov_yaw").as_double();
    vslam_xy_ = get_parameter("vslam_pos_cov_xy").as_double();
    vslam_yaw_ = get_parameter("vslam_pos_cov_yaw").as_double();
    imu_orient_ = get_parameter("imu_orient_cov").as_double();
    imu_gyro_ = get_parameter("imu_gyro_cov").as_double();
    imu_accel_ = get_parameter("imu_accel_cov").as_double();

    auto best_effort = rclcpp::SensorDataQoS();

    // Wheel odometry relay
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "wheel_odom_raw", best_effort,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        fill_odom_covariance(*msg, odom_xy_, odom_yaw_, odom_tw_xy_, odom_tw_yaw_);
        odom_pub_->publish(*msg);
      });

    // Visual SLAM odometry relay (absolute path — not namespaced)
    vslam_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry_cov", 10);
    vslam_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry", best_effort,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        fill_odom_covariance(*msg, vslam_xy_, vslam_yaw_, vslam_xy_, vslam_yaw_);
        vslam_pub_->publish(*msg);
      });

    // IMU relay
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_raw", best_effort,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        fill_imu_covariance(*msg);
        imu_pub_->publish(*msg);
      });

    RCLCPP_INFO(get_logger(),
      "Covariance override: odom(xy=%.4f,yaw=%.4f) imu(orient=%.4f,gyro=%.4f)",
      odom_xy_, odom_yaw_, imu_orient_, imu_gyro_);
  }

private:
  void fill_odom_covariance(nav_msgs::msg::Odometry& msg,
    double xy, double yaw, double tw_xy, double tw_yaw)
  {
    // Only override if covariance is all zeros
    bool all_zero = true;
    for (int i : {0, 7, 35}) {
      if (std::abs(msg.pose.covariance[i]) > 1e-12) { all_zero = false; break; }
    }
    if (!all_zero) return; // Real robot publishes nonzero — don't override

    // Pose covariance (6x6 diagonal)
    msg.pose.covariance[0]  = xy;     // x
    msg.pose.covariance[7]  = xy;     // y
    msg.pose.covariance[14] = 1e6;    // z (unused in 2D)
    msg.pose.covariance[21] = 1e6;    // roll (unused)
    msg.pose.covariance[28] = 1e6;    // pitch (unused)
    msg.pose.covariance[35] = yaw;    // yaw

    // Twist covariance
    msg.twist.covariance[0]  = tw_xy;
    msg.twist.covariance[7]  = tw_xy;
    msg.twist.covariance[14] = 1e6;
    msg.twist.covariance[21] = 1e6;
    msg.twist.covariance[28] = 1e6;
    msg.twist.covariance[35] = tw_yaw;
  }

  void fill_imu_covariance(sensor_msgs::msg::Imu& msg)
  {
    bool all_zero = true;
    for (int i : {0, 4, 8}) {
      if (std::abs(msg.orientation_covariance[i]) > 1e-12) { all_zero = false; break; }
    }
    if (!all_zero) return;

    // Orientation covariance (3x3)
    msg.orientation_covariance[0] = imu_orient_;  // roll
    msg.orientation_covariance[4] = imu_orient_;  // pitch
    msg.orientation_covariance[8] = imu_orient_;  // yaw

    // Angular velocity covariance
    msg.angular_velocity_covariance[0] = imu_gyro_;
    msg.angular_velocity_covariance[4] = imu_gyro_;
    msg.angular_velocity_covariance[8] = imu_gyro_;

    // Linear acceleration covariance
    msg.linear_acceleration_covariance[0] = imu_accel_;
    msg.linear_acceleration_covariance[4] = imu_accel_;
    msg.linear_acceleration_covariance[8] = imu_accel_;
  }

  double odom_xy_, odom_yaw_, odom_tw_xy_, odom_tw_yaw_;
  double vslam_xy_, vslam_yaw_;
  double imu_orient_, imu_gyro_, imu_accel_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vslam_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vslam_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CovarianceOverrideNode>());
  rclcpp::shutdown();
  return 0;
}
