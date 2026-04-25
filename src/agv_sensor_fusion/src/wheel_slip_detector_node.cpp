// wheel_slip_detector_node — runtime wrapper around WheelSlipDetector.
//
// Subscribes to /agv/wheel_odom, /agv/imu/filtered, and (optionally)
// /visual_slam/tracking/odometry. Publishes /agv/wheel_odom_validated
// with the input message's content and a covariance that is dynamically
// inflated when the detector flags slip. Also publishes
// /agv/wheel_slip/state as a std_msgs/String JSON for diagnostics.
//
// Configure ekf_local.yaml to consume `/agv/wheel_odom_validated` in
// place of `/agv/wheel_odom` to take advantage of the slip rejection.
// Phase 2 of the calibration plan, see docs/calibration/baseline_protocol.md.

#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "agv_sensor_fusion/wheel_slip_detector.hpp"

namespace {
// Tiny JSON state extractor: looks for "state":"VALUE" in the payload.
// Avoids pulling in a JSON dependency for a single-field read.
std::string extract_json_state(const std::string& s) {
  const std::string key = "\"state\":\"";
  auto p = s.find(key);
  if (p == std::string::npos) return "";
  p += key.size();
  auto e = s.find('"', p);
  if (e == std::string::npos) return "";
  return s.substr(p, e - p);
}
}  // namespace

namespace {
double monotonic_now() {
  return rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
}
}  // namespace

namespace agv_sensor_fusion {

class WheelSlipDetectorNode : public rclcpp::Node {
 public:
  WheelSlipDetectorNode() : rclcpp::Node("wheel_slip_detector"), detector_(load_params()) {
    using std::placeholders::_1;

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_wheel_ = create_subscription<nav_msgs::msg::Odometry>(
        "/agv/wheel_odom", sensor_qos,
        std::bind(&WheelSlipDetectorNode::on_wheel, this, _1));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "/agv/imu/filtered", sensor_qos,
        std::bind(&WheelSlipDetectorNode::on_imu, this, _1));

    sub_visual_ = create_subscription<nav_msgs::msg::Odometry>(
        "/visual_slam/tracking/odometry", sensor_qos,
        std::bind(&WheelSlipDetectorNode::on_visual, this, _1));

    // Phase 2.5: subscribe to the caster dwell advisor for the
    // structural signal. While DWELLING is active, force inflate.
    sub_dwell_ = create_subscription<std_msgs::msg::String>(
        "/agv/caster/dwell_state", reliable_qos,
        std::bind(&WheelSlipDetectorNode::on_dwell, this, _1));

    pub_validated_ = create_publisher<nav_msgs::msg::Odometry>(
        "/agv/wheel_odom_validated", reliable_qos);

    pub_state_ = create_publisher<std_msgs::msg::String>(
        "/agv/wheel_slip/state", reliable_qos);

    RCLCPP_INFO(get_logger(),
                "wheel_slip_detector ready: τ_yaw=%.3f rad/s, τ_vx=%.3f m/s, "
                "min_active=%.2fs, settle=%.2fs, require_visual=%s",
                detector_.params().yaw_rate_threshold_rad_s,
                detector_.params().linear_velocity_threshold_m_s,
                detector_.params().min_active_s,
                detector_.params().settle_s,
                detector_.params().require_visual ? "true" : "false");
  }

 private:
  WheelSlipDetectorParams load_params() {
    WheelSlipDetectorParams p;
    p.yaw_rate_threshold_rad_s = declare_parameter<double>(
        "yaw_rate_threshold_rad_s", p.yaw_rate_threshold_rad_s);
    p.linear_velocity_threshold_m_s = declare_parameter<double>(
        "linear_velocity_threshold_m_s", p.linear_velocity_threshold_m_s);
    p.min_active_s = declare_parameter<double>("min_active_s", p.min_active_s);
    p.settle_s = declare_parameter<double>("settle_s", p.settle_s);
    p.imu_max_age_s = declare_parameter<double>("imu_max_age_s", p.imu_max_age_s);
    p.visual_max_age_s = declare_parameter<double>("visual_max_age_s", p.visual_max_age_s);
    p.require_visual = declare_parameter<bool>("require_visual", p.require_visual);
    p.inflated_xx = declare_parameter<double>("inflated_xx", p.inflated_xx);
    p.baseline_xx = declare_parameter<double>("baseline_xx", p.baseline_xx);
    p.forward_upstream_baseline = declare_parameter<bool>(
        "forward_upstream_baseline", p.forward_upstream_baseline);
    return p;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_wz_ = msg->angular_velocity.z;
    last_imu_t_ = monotonic_now();
  }

  void on_visual(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_visual_vx_ = msg->twist.twist.linear.x;
    last_visual_t_ = monotonic_now();
  }

  void on_dwell(const std_msgs::msg::String::SharedPtr msg) {
    last_dwell_active_ = (extract_json_state(msg->data) == "DWELLING");
  }

  void on_wheel(const nav_msgs::msg::Odometry::SharedPtr msg) {
    SlipObservation obs{};
    obs.t_now = monotonic_now();
    obs.wheel_vx = msg->twist.twist.linear.x;
    obs.wheel_wz = msg->twist.twist.angular.z;
    obs.imu_wz = last_imu_wz_;
    obs.t_imu_last = last_imu_t_;
    if (last_visual_t_ > 0.0) {
      obs.visual_vx = last_visual_vx_;
      obs.t_visual_last = last_visual_t_;
    } else {
      obs.t_visual_last = std::numeric_limits<double>::quiet_NaN();
    }
    obs.dwell_active = last_dwell_active_;

    SlipDecision dec = detector_.step(obs);

    // Build validated odometry: copy the input message and override the
    // covariance entries the EKF cares about.
    auto out = *msg;
    if (dec.inflate_covariance) {
      // Inflate the linear-x and angular-z velocity covariances. The
      // EKF (robot_localization with twist_config0 trusting linear x
      // and angular z) reads these.
      out.twist.covariance[0]  = detector_.params().inflated_xx;   // vx
      out.twist.covariance[35] = detector_.params().inflated_xx;   // wz
    } else if (!detector_.params().forward_upstream_baseline) {
      out.twist.covariance[0]  = detector_.params().baseline_xx;
      out.twist.covariance[35] = detector_.params().baseline_xx;
    }
    pub_validated_->publish(out);

    // Publish state JSON for diagnostics
    std_msgs::msg::String state_msg;
    std::ostringstream js;
    js << "{\"state\":\"" << to_string(dec.state) << "\","
       << "\"residual_wz\":" << dec.residual_yaw_rate << ",";
    if (dec.residual_vx.has_value()) {
      js << "\"residual_vx\":" << *dec.residual_vx << ",";
    } else {
      js << "\"residual_vx\":null,";
    }
    js << "\"inflated\":" << (dec.inflate_covariance ? "true" : "false") << ","
       << "\"reason\":\"" << dec.reason << "\"}";
    state_msg.data = js.str();
    pub_state_->publish(state_msg);
  }

  WheelSlipDetector detector_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_visual_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_dwell_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_validated_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;

  double last_imu_wz_ = 0.0;
  double last_imu_t_ = 0.0;
  double last_visual_vx_ = 0.0;
  double last_visual_t_ = 0.0;
  bool last_dwell_active_ = false;
};

}  // namespace agv_sensor_fusion

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_sensor_fusion::WheelSlipDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
