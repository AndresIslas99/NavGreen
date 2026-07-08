// rail_driver_node — longitudinal-only drive controller for the AGV.
//
// Consumes a goal pose and publishes cmd_vel with angular.z == 0 hard.
// Enforces lateral abort + yaw abort (when in rail zone) + collision halt.
//
// Inputs:
//   /agv/odometry/global          (nav_msgs/Odometry)   — current pose
//   /agv/rail_driver/goal         (geometry_msgs/PoseStamped) — target pose
//   /agv/zone/state               (std_msgs/String JSON) — from zone_detector
//   /agv/collision_monitor_state  (std_msgs/String)     — "stop"/"slowdown"/"clear"
//
// Outputs:
//   /agv/cmd_vel_rail             (geometry_msgs/Twist) — longitudinal-only
//   /agv/rail_driver/state        (std_msgs/String JSON) — state + remaining_m

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "agv_rail_driver/rail_controller.hpp"

using std::placeholders::_1;

namespace agv_rail_driver {

class RailDriverNode : public rclcpp::Node {
 public:
  RailDriverNode() : rclcpp::Node("rail_driver") {
    declare_parameter<std::string>("odom_topic", "/agv/odometry/global");
    declare_parameter<std::string>("goal_topic", "/agv/rail_driver/goal");
    declare_parameter<std::string>("zone_topic", "/agv/zone/state");
    declare_parameter<std::string>("collision_topic", "/agv/collision_monitor_state");
    declare_parameter<std::string>("cmd_vel_topic", "/agv/cmd_vel_rail");
    declare_parameter<std::string>("state_topic", "/agv/rail_driver/state");
    declare_parameter<std::string>("rail_detections_topic", "/agv/rail_detections");
    declare_parameter<std::string>("rail_detector_state_topic",
                                   "/agv/rail_detector/state");
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("kP", 1.0);
    declare_parameter<double>("speed_max_mps", 1.0);
    declare_parameter<double>("stop_band_m", 0.05);
    declare_parameter<double>("lateral_abort_m", 0.30);
    declare_parameter<double>("yaw_abort_rad", 0.26);
    declare_parameter<double>("visual_min_conf", 0.7);
    declare_parameter<double>("visual_max_age_s", 0.5);

    params_.kP              = get_parameter("kP").as_double();
    params_.speed_max_mps   = get_parameter("speed_max_mps").as_double();
    params_.stop_band_m     = get_parameter("stop_band_m").as_double();
    params_.lateral_abort_m = get_parameter("lateral_abort_m").as_double();
    params_.yaw_abort_rad   = get_parameter("yaw_abort_rad").as_double();
    params_.visual_min_conf  = get_parameter("visual_min_conf").as_double();
    params_.visual_max_age_s = get_parameter("visual_max_age_s").as_double();

    const auto odom_topic      = get_parameter("odom_topic").as_string();
    const auto goal_topic      = get_parameter("goal_topic").as_string();
    const auto zone_topic      = get_parameter("zone_topic").as_string();
    const auto collision_topic = get_parameter("collision_topic").as_string();
    const auto cmd_vel_topic   = get_parameter("cmd_vel_topic").as_string();
    const auto state_topic     = get_parameter("state_topic").as_string();
    const double rate          = get_parameter("publish_rate_hz").as_double();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS{10},
        std::bind(&RailDriverNode::on_odom, this, _1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, rclcpp::QoS{1},
        std::bind(&RailDriverNode::on_goal, this, _1));
    sub_zone_ = create_subscription<std_msgs::msg::String>(
        zone_topic, rclcpp::QoS{10},
        std::bind(&RailDriverNode::on_zone, this, _1));
    sub_collision_ = create_subscription<std_msgs::msg::String>(
        collision_topic, rclcpp::QoS{10},
        std::bind(&RailDriverNode::on_collision, this, _1));
    sub_rail_detections_ = create_subscription<geometry_msgs::msg::PoseArray>(
        get_parameter("rail_detections_topic").as_string(),
        rclcpp::QoS{5},
        std::bind(&RailDriverNode::on_rail_detections, this, _1));
    sub_rail_detector_state_ = create_subscription<std_msgs::msg::String>(
        get_parameter("rail_detector_state_topic").as_string(),
        rclcpp::QoS{5},
        std::bind(&RailDriverNode::on_rail_detector_state, this, _1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS{10});
    // Iter-26 Bug B: state publisher depth=1 with reliable delivery so
    // subscribers never see a buffered stale "reached"/"driving" from a
    // previous waypoint. iter-25 c2_reverse_exit / c4_reverse_exit
    // reported SUCCEEDED in 0.0 s because the harness's first spin_once
    // after the new goal drained a buffered "reached" from the preceding
    // drive_in. Mirrors the iter-20 fix applied to rail_approach's
    // state publisher.
    rclcpp::QoS state_qos{rclcpp::KeepLast(1)};
    state_qos.reliable();
    pub_state_ = create_publisher<std_msgs::msg::String>(state_topic, state_qos);

    // Iter-22: cancel_goal service — atomic, synchronous cancel of any
    // active rail goal. Replaces the iter-7/8/9 "publish a zero-distance
    // PoseStamped at the current pose" hack in the test harness, which
    // had a race with the mode_arbiter's auto-EXIT_PUSH (observed as
    // `clearance_now=-2.95` and the robot drifting 2.95 m before the
    // teleport took effect). The service invalidates have_goal_ in the
    // service callback, publishes a one-shot CANCELED state tick via
    // cancel_requested_, and stops the robot (cmd_vel=0). The next
    // on_tick() sees have_goal=false and compute() returns IDLE.
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
        "/agv/rail_driver/cancel_goal",
        std::bind(&RailDriverNode::on_cancel_goal, this,
                  std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        std::bind(&RailDriverNode::on_tick, this));

    RCLCPP_INFO(get_logger(),
                "rail_driver up: odom=%s goal=%s → cmd=%s @ %.1f Hz "
                "(kP=%.2f speed_max=%.2f stop_band=%.3f)",
                odom_topic.c_str(), goal_topic.c_str(),
                cmd_vel_topic.c_str(), rate,
                params_.kP, params_.speed_max_mps, params_.stop_band_m);
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    last_odom_ = msg;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    have_goal_ = true;
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    RCLCPP_INFO(get_logger(), "new goal: (%.3f, %.3f)", goal_x_, goal_y_);
    // Iter-26 Bug B: publish an immediate "driving" state tick so the
    // harness's `_wait_for_state_value` cannot latch onto the last
    // "reached" from the previous goal. Without this the subscriber's
    // keep-last-1 buffer may still hold the stale message until the
    // next on_tick() (up to 50 ms), which is enough for a fast dispatch
    // on a continuous-flow waypoint to false-positive.
    double remaining = 0.0;
    if (last_odom_) {
      remaining = std::hypot(
          goal_x_ - last_odom_->pose.pose.position.x,
          goal_y_ - last_odom_->pose.pose.position.y);
    }
    std::ostringstream os;
    os.precision(4);
    os << std::fixed << "{\"state\":\"driving\","
       << "\"linear_x\":0.0000,"
       << "\"remaining_m\":" << remaining << ","
       << "\"in_rail_zone\":" << (last_in_rail_ ? "true" : "false") << ","
       << "\"collision_stop\":" << (last_collision_stop_ ? "true" : "false") << "}";
    std_msgs::msg::String state_msg;
    state_msg.data = os.str();
    pub_state_->publish(state_msg);
  }

  void on_zone(const std_msgs::msg::String::ConstSharedPtr msg) {
    // Minimal JSON parse: we only need rail_yaw_error and whether we're in a rail.
    const std::string &d = msg->data;
    auto extract_double = [&](const std::string &key) -> double {
      const std::string needle = "\"" + key + "\":";
      const auto pos = d.find(needle);
      if (pos == std::string::npos) return std::nan("");
      size_t p = pos + needle.size();
      if (p < d.size() && d[p] == 'n') return std::nan("");  // null
      try {
        return std::stod(d.substr(p));
      } catch (...) {
        return std::nan("");  // malformed/truncated value — not available
      }
    };
    auto extract_string = [&](const std::string &key) -> std::string {
      const std::string needle = "\"" + key + "\":\"";
      const auto pos = d.find(needle);
      if (pos == std::string::npos) return "";
      const size_t p = pos + needle.size();
      const auto end = d.find('"', p);
      if (end == std::string::npos) return "";
      return d.substr(p, end - p);
    };
    last_rail_yaw_error_ = extract_double("rail_yaw_error");
    const auto zone = extract_string("zone");
    last_in_rail_ = zone.rfind("rail_aisle_", 0) == 0;
  }

  void on_collision(const std_msgs::msg::String::ConstSharedPtr msg) {
    // collision_monitor publishes strings like "stop"/"slowdown"/"clear".
    last_collision_stop_ = (msg->data.find("stop") != std::string::npos);
  }

  // PoseArray has 2 poses, one per rail, in base_link. Midpoint Y = rail
  // centerline lateral offset; average yaw = rail axis direction vs body +X.
  void on_rail_detections(
      const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
    if (msg->poses.size() != 2) return;
    const double y0 = msg->poses[0].position.y;
    const double y1 = msg->poses[1].position.y;
    visual_lat_offset_ = 0.5 * (y0 + y1);
    auto yaw_from_q = [](const geometry_msgs::msg::Quaternion &q) {
      return std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };
    const double yaw0 = yaw_from_q(msg->poses[0].orientation);
    const double yaw1 = yaw_from_q(msg->poses[1].orientation);
    // Rails are parallel by RANSAC construction; straight average is fine so
    // long as neither has been wrapped across ±π. Both come from the same
    // direction vector in the detector so they stay on the same branch.
    visual_yaw_error_ = 0.5 * (yaw0 + yaw1);
    visual_last_stamp_ = msg->header.stamp;
    visual_have_stamp_ = true;
  }

  void on_rail_detector_state(
      const std_msgs::msg::String::ConstSharedPtr msg) {
    // Minimal JSON parse — only confidence is consumed here.
    const std::string &d = msg->data;
    const std::string needle = "\"confidence\":";
    const auto pos = d.find(needle);
    if (pos == std::string::npos) return;
    try {
      visual_confidence_ = std::stod(d.substr(pos + needle.size()));
    } catch (...) {
      visual_confidence_ = 0.0;
    }
  }

  void on_cancel_goal(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    // Synchronous drop: clear have_goal_ AND arm the one-shot CANCELED
    // tick that on_tick() will honor on its next invocation. We also
    // publish a zero cmd_vel immediately so downstream consumers
    // (collision_monitor, arbiter relay) see the stop without waiting
    // for the next 20 Hz tick.
    have_goal_ = false;
    cancel_requested_ = true;
    geometry_msgs::msg::Twist stop;
    pub_cmd_->publish(stop);
    resp->success = true;
    resp->message = "rail_driver goal canceled";
    RCLCPP_INFO(get_logger(), "cancel_goal: have_goal_=false, "
                "emitting CANCELED tick");
  }

  void on_tick() {
    if (!last_odom_) return;

    // Iter-22: if cancel was requested since last tick, emit a one-shot
    // CANCELED state message and publish cmd_vel=0 before running the
    // control law. Skips compute() entirely to avoid races with visual
    // feedback or lateral-abort checks firing on stale inputs.
    if (cancel_requested_) {
      cancel_requested_ = false;
      geometry_msgs::msg::Twist stop;
      pub_cmd_->publish(stop);
      std::ostringstream cs;
      cs.precision(4);
      cs << std::fixed << "{\"state\":\"canceled\","
         << "\"linear_x\":0.0000,"
         << "\"remaining_m\":0.0000,"
         << "\"in_rail_zone\":" << (last_in_rail_ ? "true" : "false") << ","
         << "\"collision_stop\":" << (last_collision_stop_ ? "true" : "false") << "}";
      std_msgs::msg::String cm;
      cm.data = cs.str();
      pub_state_->publish(cm);
      return;
    }

    RailControllerInputs in;
    in.current_x = last_odom_->pose.pose.position.x;
    in.current_y = last_odom_->pose.pose.position.y;
    // Extract yaw from odometry orientation so the controller can project
    // the world-frame error onto the robot's body +X.
    {
      const auto &q = last_odom_->pose.pose.orientation;
      in.current_yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    in.goal_x = goal_x_;
    in.goal_y = goal_y_;
    in.rail_axis_sign = (goal_x_ >= in.current_x) ? 1.0 : -1.0;
    in.rail_yaw_error = last_rail_yaw_error_;
    in.in_rail_zone = last_in_rail_;
    in.collision_monitor_stop = last_collision_stop_;
    in.have_goal = have_goal_;

    in.visual_lat_offset = visual_lat_offset_;
    in.visual_yaw_error  = visual_yaw_error_;
    in.visual_confidence = visual_confidence_;
    if (visual_have_stamp_) {
      in.visual_age_s =
          (get_clock()->now() - rclcpp::Time(visual_last_stamp_)).seconds();
    } else {
      in.visual_age_s = std::numeric_limits<double>::infinity();
    }

    const auto out = compute(in, params_);

    // Iter-31 diagnostics: log the FIRST tick on which a BLOCKED_* fires,
    // with the gate reason, pose, visual snapshot, and pose snapshot. The
    // next iter-31 run then answers the iter-28..30 mystery of *which*
    // gate is actually tripping on c3/c5 drive_in. Suppressed once state
    // sticks so we don't spam the log at 20 Hz while held in BLOCKED.
    const bool is_blocked = (out.state == RailState::BLOCKED_LATERAL ||
                             out.state == RailState::BLOCKED_MISALIGNED ||
                             out.state == RailState::BLOCKED_WAIT);
    if (is_blocked && out.state != last_state_logged_) {
      RCLCPP_WARN(get_logger(),
          "BLOCKED %s (reason=%s) at pose=(%.3f, %.3f, yaw=%.3f) "
          "goal=(%.3f, %.3f) visual={lat=%.3f yaw=%.3f conf=%.2f age=%.2fs} "
          "pose_lat_err=%.3f rail_yaw_err=%.3f in_rail_zone=%d",
          state_to_str(out.state), out.block_reason,
          in.current_x, in.current_y, in.current_yaw,
          in.goal_x, in.goal_y,
          in.visual_lat_offset, in.visual_yaw_error,
          in.visual_confidence, in.visual_age_s,
          (in.current_y - in.goal_y), in.rail_yaw_error,
          in.in_rail_zone ? 1 : 0);
    }
    last_state_logged_ = out.state;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.linear_x;
    cmd.angular.z = 0.0;  // defensive — the controller already enforces this
    pub_cmd_->publish(cmd);

    std::ostringstream os;
    os.precision(4);
    os << std::fixed << "{\"state\":\"" << state_to_str(out.state) << "\","
       << "\"linear_x\":" << out.linear_x << ","
       << "\"remaining_m\":" << out.remaining_m << ","
       << "\"in_rail_zone\":" << (last_in_rail_ ? "true" : "false") << ","
       << "\"collision_stop\":" << (last_collision_stop_ ? "true" : "false") << "}";
    std_msgs::msg::String state_msg;
    state_msg.data = os.str();
    pub_state_->publish(state_msg);

    if (out.state == RailState::REACHED && have_goal_) {
      RCLCPP_INFO(get_logger(), "goal reached (err=%.3f m)", out.remaining_m);
      have_goal_ = false;  // Latch to IDLE until next goal.
    }
  }

  RailControllerParams params_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_zone_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_collision_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_rail_detections_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_rail_detector_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  bool have_goal_ = false;
  bool cancel_requested_ = false;   // Iter-22: set by cancel_goal service.
  RailState last_state_logged_ = RailState::IDLE;  // Iter-31: edge-trigger WARN
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double last_rail_yaw_error_ = std::nan("");
  bool last_in_rail_ = false;
  bool last_collision_stop_ = false;

  double visual_lat_offset_ = 0.0;
  double visual_yaw_error_  = 0.0;
  double visual_confidence_ = 0.0;
  builtin_interfaces::msg::Time visual_last_stamp_;
  bool   visual_have_stamp_ = false;
};

}  // namespace agv_rail_driver

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_rail_driver::RailDriverNode>());
  rclcpp::shutdown();
  return 0;
}
