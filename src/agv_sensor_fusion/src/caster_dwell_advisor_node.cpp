// caster_dwell_advisor_node — wraps CasterDwellAdvisor.
//
// Subscribes to /agv/cmd_vel (the commanded twist) and publishes
// /agv/caster/dwell_state (std_msgs/String JSON). This is a passive
// observer; closing the loop requires either a controller change or
// a middleware that gates cmd_vel during the dwell window.
//
// Phase 4 of the diff-drive calibration plan, advisory variant.

#include <cmath>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include "agv_sensor_fusion/caster_dwell_advisor.hpp"

namespace agv_sensor_fusion {

class CasterDwellAdvisorNode : public rclcpp::Node {
 public:
  CasterDwellAdvisorNode()
      : rclcpp::Node("caster_dwell_advisor"),
        advisor_(load_params()) {
    using std::placeholders::_1;
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "/agv/cmd_vel", reliable_qos,
        std::bind(&CasterDwellAdvisorNode::on_cmd, this, _1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/agv/wheel_odom", sensor_qos,
        std::bind(&CasterDwellAdvisorNode::on_odom, this, _1));

    pub_state_ = create_publisher<std_msgs::msg::String>(
        "/agv/caster/dwell_state", reliable_qos);

    // Periodic publish at 20 Hz so consumers always have a fresh value
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&CasterDwellAdvisorNode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "caster_dwell_advisor ready: deadband=%.3f m/s, dwell=%.2fs",
                advisor_.params().deadband_vx_m_s,
                advisor_.params().dwell_s);
  }

 private:
  CasterDwellParams load_params() {
    CasterDwellParams p;
    p.deadband_vx_m_s = declare_parameter<double>("deadband_vx_m_s", p.deadband_vx_m_s);
    p.dwell_s = declare_parameter<double>("dwell_s", p.dwell_s);
    return p;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_vx_ = msg->linear.x;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_meas_vx_ = msg->twist.twist.linear.x;
  }

  void on_timer() {
    CasterObservation obs{};
    obs.t_now = rclcpp::Clock(RCL_STEADY_TIME).now().seconds();
    obs.cmd_vx = last_cmd_vx_;
    obs.measured_vx = last_meas_vx_;
    auto advice = advisor_.step(obs);

    std_msgs::msg::String msg;
    std::ostringstream js;
    js << "{\"state\":\"" << to_string(advice.state) << "\","
       << "\"last_sign\":" << advice.last_sign << ","
       << "\"seconds_remaining\":" << advice.seconds_remaining << "}";
    msg.data = js.str();
    pub_state_->publish(msg);
  }

  CasterDwellAdvisor advisor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_cmd_vx_ = 0.0;
  double last_meas_vx_ = 0.0;
};

}  // namespace agv_sensor_fusion

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_sensor_fusion::CasterDwellAdvisorNode>());
  rclcpp::shutdown();
  return 0;
}
