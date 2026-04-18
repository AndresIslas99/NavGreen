// zone_detector_node — publishes current greenhouse zone based on robot pose.
//
// Reads: /agv/odometry/global (nav_msgs/Odometry, map frame)
// Writes: /agv/zone/state (std_msgs/String, JSON)
//
// JSON payload:
//   {"zone": str, "section": str, "aisle_y_center": float or null,
//    "rail_offset_lat": float or null, "rail_yaw_error": float or null,
//    "confidence": float, "source": "pose"}
//
// This is the Phase 1 fallback source. Phase 2 will add apriltag + ZED
// visual sources that override when available (see rail_driver_spec.md).

#include <chrono>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_zone_detector/zone_classifier.hpp"
#include "agv_zone_detector/zone_classifier_impl.hpp"

using std::placeholders::_1;

namespace agv_zone_detector {

class ZoneDetectorNode : public rclcpp::Node {
 public:
  ZoneDetectorNode() : rclcpp::Node("zone_detector") {
    declare_parameter<std::string>("odom_topic", "/agv/odometry/global");
    declare_parameter<std::string>("zone_topic", "/agv/zone/state");
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("aisle_half_width", 0.35);

    odom_topic_ = get_parameter("odom_topic").as_string();
    zone_topic_ = get_parameter("zone_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    aisle_half_width_ = get_parameter("aisle_half_width").as_double();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS{10},
        std::bind(&ZoneDetectorNode::on_odom, this, _1));

    pub_zone_ = create_publisher<std_msgs::msg::String>(zone_topic_, rclcpp::QoS{10});

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ZoneDetectorNode::on_tick, this));

    RCLCPP_INFO(get_logger(),
                "zone_detector: %s -> %s @ %.1f Hz (aisle_half_width=%.2f m)",
                odom_topic_.c_str(), zone_topic_.c_str(),
                publish_rate_hz_, aisle_half_width_);
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    last_msg_ = msg;
  }

  void on_tick() {
    if (!last_msg_) return;
    const auto &p = last_msg_->pose.pose.position;
    const auto &q = last_msg_->pose.pose.orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const auto res = classify(p.x, p.y, yaw, aisle_half_width_);
    publish(res);
  }

  static std::string to_json_num_or_null(double v) {
    if (std::isnan(v)) return "null";
    std::ostringstream os;
    os.precision(6);
    os << std::fixed << v;
    return os.str();
  }

  void publish(const ClassifyResult &r) {
    std::ostringstream os;
    os.precision(4);
    // approach_tag_id is -1 when the zone is not an approach; emit null.
    std::string tag_id_str =
        (r.approach_tag_id < 0) ? "null" : std::to_string(r.approach_tag_id);
    os << std::fixed
       << "{\"zone\":\""      << r.zone << "\","
       << "\"section\":\""   << r.section << "\","
       << "\"aisle_y_center\":"   << to_json_num_or_null(r.aisle_y_center) << ","
       << "\"rail_offset_lat\":"  << to_json_num_or_null(r.rail_offset_lat) << ","
       << "\"rail_yaw_error\":"   << to_json_num_or_null(r.rail_yaw_error) << ","
       << "\"approach_tag_id\":"  << tag_id_str << ","
       << "\"confidence\":"  << r.confidence << ","
       << "\"source\":\"pose\"}";

    std_msgs::msg::String out;
    out.data = os.str();
    pub_zone_->publish(out);
  }

  std::string odom_topic_;
  std::string zone_topic_;
  double publish_rate_hz_;
  double aisle_half_width_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_zone_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_msg_;
};

}  // namespace agv_zone_detector

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_zone_detector::ZoneDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
