/**
 * IMU vibration filter — Butterworth low-pass on gyro and accel.
 *
 * Greenhouse floor vibrations transmit through the chassis to the ZED 2i IMU,
 * corrupting angular velocity and acceleration readings. This node filters
 * those high-frequency disturbances while preserving the robot's real dynamics.
 *
 * Orientation (quaternion) passes through unfiltered — it's already integrated
 * by the ZED SDK and filtering it would add unacceptable lag.
 *
 * Subscribe: raw IMU topic (e.g. /agv/zed/imu/data)
 * Publish:   filtered IMU topic (e.g. /agv/imu/filtered)
 */

#include "agv_sensor_fusion/imu_filter_node.hpp"
#include <cmath>

namespace agv_sensor_fusion {

// ── Butterworth2 ──

void Butterworth2::configure(double sample_rate, double cutoff_hz) {
  // Bilinear transform: pre-warp analog cutoff to digital
  double wc = std::tan(M_PI * cutoff_hz / sample_rate);
  double wc2 = wc * wc;
  double sqrt2_wc = std::sqrt(2.0) * wc;
  double norm = 1.0 / (1.0 + sqrt2_wc + wc2);

  b0_ = wc2 * norm;
  b1_ = 2.0 * b0_;
  b2_ = b0_;
  a1_ = 2.0 * (wc2 - 1.0) * norm;
  a2_ = (1.0 - sqrt2_wc + wc2) * norm;

  reset();
}

double Butterworth2::apply(double x) {
  double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
  x2_ = x1_;
  x1_ = x;
  y2_ = y1_;
  y1_ = y;
  return y;
}

void Butterworth2::reset() {
  x1_ = x2_ = y1_ = y2_ = 0.0;
}

// ── ImuFilterNode ──

ImuFilterNode::ImuFilterNode() : Node("imu_filter") {
  declare_parameter("gyro_cutoff_hz", 10.0);
  declare_parameter("accel_cutoff_hz", 5.0);
  declare_parameter("sample_rate", 200.0);

  gyro_cutoff_hz_ = get_parameter("gyro_cutoff_hz").as_double();
  accel_cutoff_hz_ = get_parameter("accel_cutoff_hz").as_double();
  sample_rate_ = get_parameter("sample_rate").as_double();

  for (auto& f : gyro_filters_)  f.configure(sample_rate_, gyro_cutoff_hz_);
  for (auto& f : accel_filters_) f.configure(sample_rate_, accel_cutoff_hz_);

  sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/raw", rclcpp::SensorDataQoS(),
    std::bind(&ImuFilterNode::imu_callback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "imu/filtered", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(),
    "IMU filter: gyro %.0f Hz, accel %.0f Hz cutoff (Butterworth 2nd order, %.0f Hz sample rate)",
    gyro_cutoff_hz_, accel_cutoff_hz_, sample_rate_);
}

void ImuFilterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Seed filters with first sample to avoid transient
  if (!initialized_) {
    initialized_ = true;
    for (int i = 0; i < 3; ++i) {
      double gv = (i == 0) ? msg->angular_velocity.x
                 : (i == 1) ? msg->angular_velocity.y
                            : msg->angular_velocity.z;
      double av = (i == 0) ? msg->linear_acceleration.x
                 : (i == 1) ? msg->linear_acceleration.y
                            : msg->linear_acceleration.z;
      // Pre-fill filter history with first sample (avoids step response transient)
      for (int j = 0; j < 10; ++j) {
        gyro_filters_[i].apply(gv);
        accel_filters_[i].apply(av);
      }
    }
  }

  auto out = std::make_unique<sensor_msgs::msg::Imu>(*msg);

  // Filter angular velocity (gyro) — removes vibration noise
  out->angular_velocity.x = gyro_filters_[0].apply(msg->angular_velocity.x);
  out->angular_velocity.y = gyro_filters_[1].apply(msg->angular_velocity.y);
  out->angular_velocity.z = gyro_filters_[2].apply(msg->angular_velocity.z);

  // Filter linear acceleration — removes vibration noise
  out->linear_acceleration.x = accel_filters_[0].apply(msg->linear_acceleration.x);
  out->linear_acceleration.y = accel_filters_[1].apply(msg->linear_acceleration.y);
  out->linear_acceleration.z = accel_filters_[2].apply(msg->linear_acceleration.z);

  // Orientation passes through unfiltered (already integrated by ZED SDK)

  pub_->publish(std::move(out));
}

}  // namespace agv_sensor_fusion

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_sensor_fusion::ImuFilterNode>());
  rclcpp::shutdown();
  return 0;
}
