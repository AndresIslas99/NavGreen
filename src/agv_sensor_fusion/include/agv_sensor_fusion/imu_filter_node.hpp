#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace agv_sensor_fusion {

/// 2nd-order Butterworth low-pass filter (single channel).
/// Coefficients computed at construction from sample rate and cutoff frequency.
class Butterworth2 {
public:
  Butterworth2() = default;
  void configure(double sample_rate, double cutoff_hz);
  double apply(double x);
  void reset();

private:
  // Transfer function coefficients: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
  //                                       - a1*y[n-1] - a2*y[n-2]
  double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0;
  double a1_ = 0.0, a2_ = 0.0;
  double x1_ = 0.0, x2_ = 0.0;  // input history
  double y1_ = 0.0, y2_ = 0.0;  // output history
};

/// Filters IMU vibrations using Butterworth low-pass on angular velocity
/// and linear acceleration. Orientation (quaternion) passes through unfiltered.
class ImuFilterNode : public rclcpp::Node {
public:
  ImuFilterNode();

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  // Per-axis filters: [x, y, z]
  std::array<Butterworth2, 3> gyro_filters_;
  std::array<Butterworth2, 3> accel_filters_;

  double gyro_cutoff_hz_;
  double accel_cutoff_hz_;
  double sample_rate_;
  bool initialized_ = false;
};

}  // namespace agv_sensor_fusion
