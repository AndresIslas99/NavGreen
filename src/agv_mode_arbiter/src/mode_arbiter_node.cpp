// mode_arbiter_node — owns /agv/cmd_vel in the 3-mode architecture.
//
// Ticks at 20 Hz. Each tick:
//   1. Read the latest observation (zone, rail_approach state, rail_driver
//      state, safety signal, operator mode).
//   2. Advance the FSM (mode_fsm.hpp).
//   3. Relay the latest Twist from the selected source topic to /agv/cmd_vel,
//      or publish zero Twist when the source is NONE.
//   4. Publish /agv/mode/state describing the current mode + source.
//   5. Fire one-shot events (approach service call, rail_drive goal) on FSM
//      output requests.
//
// The actual rail_approach service call and rail_drive goal publication are
// stubs for now — they log the intent. Full plumbing arrives with P2.S4/S6
// once both controllers expose the expected APIs on their state topics.

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_interfaces/srv/rail_approach.hpp"

#include "agv_mode_arbiter/mode_fsm.hpp"

using std::placeholders::_1;

namespace agv_mode_arbiter {

class ModeArbiterNode : public rclcpp::Node {
 public:
  ModeArbiterNode() : rclcpp::Node("mode_arbiter") {
    declare_parameter<std::string>("cmd_vel_out_topic", "/agv/cmd_vel");
    declare_parameter<std::string>("cmd_vel_nav_topic", "/agv/cmd_vel_nav");
    declare_parameter<std::string>("cmd_vel_approach_topic",
                                    "/agv/cmd_vel_approach");
    declare_parameter<std::string>("cmd_vel_rail_topic", "/agv/cmd_vel_rail");
    declare_parameter<std::string>("zone_topic", "/agv/zone/state");
    declare_parameter<std::string>("rail_approach_state_topic",
                                    "/agv/rail_approach/state");
    declare_parameter<std::string>("rail_driver_state_topic",
                                    "/agv/rail_driver/state");
    declare_parameter<std::string>("collision_topic",
                                    "/agv/collision_monitor_state");
    declare_parameter<std::string>("operator_mode_topic", "/agv/mode/set");
    declare_parameter<std::string>("state_topic", "/agv/mode/state");
    declare_parameter<std::string>("odom_topic", "/agv/odometry/global");
    declare_parameter<std::string>("rail_goal_topic", "/agv/rail_driver/goal");
    declare_parameter<std::string>("rail_approach_service", "/agv/rail_approach/execute");
    declare_parameter<double>("publish_rate_hz", 20.0);
    // Distance to drive into the rail once the approach declares settled.
    // 3 m keeps the robot inside the 20 m rail section and inside the
    // usable traversal band (operator-confirmed geometry).
    declare_parameter<double>("rail_drive_distance_m", 3.0);
    // Distance past the exit AprilTag that the RAIL_EXIT push goal targets.
    // Must exceed the FSM's `rail_exit_clearance_m >= 1.0` release threshold
    // by a comfortable margin so rail_driver declares "reached" beyond the
    // 1 m no-rotation line (prevents a corner case where the extension goal
    // latches "reached" before the FSM is ready to release RAIL_EXIT).
    declare_parameter<double>("rail_exit_push_m", 1.5);
    // Camera-to-tag desired forward offset when requesting rail_approach.
    declare_parameter<double>("approach_offset_x", 0.3);
    declare_parameter<double>("approach_offset_y", 0.0);
    // Gate: if false, the arbiter never auto-calls rail_approach on zone
    // entry. It still relays the right cmd_vel source when rail_approach
    // is triggered externally (operator, test harness). Default false so
    // the test_waypoint_precision Nav2-direct path keeps working.
    declare_parameter<bool>("auto_approach", false);

    const auto cmd_out      = get_parameter("cmd_vel_out_topic").as_string();
    const auto cmd_nav      = get_parameter("cmd_vel_nav_topic").as_string();
    const auto cmd_approach = get_parameter("cmd_vel_approach_topic").as_string();
    const auto cmd_rail     = get_parameter("cmd_vel_rail_topic").as_string();
    const auto zone_topic   = get_parameter("zone_topic").as_string();
    const auto approach_state_topic =
        get_parameter("rail_approach_state_topic").as_string();
    const auto driver_state_topic =
        get_parameter("rail_driver_state_topic").as_string();
    const auto collision_topic  = get_parameter("collision_topic").as_string();
    const auto operator_topic   = get_parameter("operator_mode_topic").as_string();
    const auto state_topic      = get_parameter("state_topic").as_string();
    const auto odom_topic       = get_parameter("odom_topic").as_string();
    const auto rail_goal_topic  = get_parameter("rail_goal_topic").as_string();
    const auto rail_approach_service =
        get_parameter("rail_approach_service").as_string();
    const double rate           = get_parameter("publish_rate_hz").as_double();
    rail_drive_distance_m_ = get_parameter("rail_drive_distance_m").as_double();
    rail_exit_push_m_ = get_parameter("rail_exit_push_m").as_double();
    approach_offset_x_ = get_parameter("approach_offset_x").as_double();
    approach_offset_y_ = get_parameter("approach_offset_y").as_double();
    auto_approach_ = get_parameter("auto_approach").as_bool();
    latest_inputs_.auto_approach = auto_approach_;

    pub_cmd_   = create_publisher<geometry_msgs::msg::Twist>(cmd_out, rclcpp::QoS{10});
    pub_state_ = create_publisher<std_msgs::msg::String>(state_topic, rclcpp::QoS{10});
    pub_rail_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        rail_goal_topic, rclcpp::QoS{1});
    approach_client_ = create_client<agv_interfaces::srv::RailApproach>(
        rail_approach_service);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS{10},
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          latest_inputs_.current_x = msg->pose.pose.position.x;
          latest_inputs_.current_y = msg->pose.pose.position.y;
        });

    sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_nav, rclcpp::QoS{10},
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { last_nav_ = msg; });
    sub_approach_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_approach, rclcpp::QoS{10},
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { last_approach_ = msg; });
    sub_rail_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_rail, rclcpp::QoS{10},
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { last_rail_ = msg; });

    sub_zone_ = create_subscription<std_msgs::msg::String>(
        zone_topic, rclcpp::QoS{10},
        std::bind(&ModeArbiterNode::on_zone, this, _1));
    sub_approach_state_ = create_subscription<std_msgs::msg::String>(
        approach_state_topic, rclcpp::QoS{10},
        std::bind(&ModeArbiterNode::on_approach_state, this, _1));
    sub_driver_state_ = create_subscription<std_msgs::msg::String>(
        driver_state_topic, rclcpp::QoS{10},
        std::bind(&ModeArbiterNode::on_driver_state, this, _1));
    sub_collision_ = create_subscription<std_msgs::msg::String>(
        collision_topic, rclcpp::QoS{10},
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          latest_inputs_.safety_stop = (msg->data.find("stop") != std::string::npos);
        });
    sub_operator_ = create_subscription<std_msgs::msg::String>(
        operator_topic, rclcpp::QoS{10},
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          latest_inputs_.operator_mode = msg->data;
        });

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        std::bind(&ModeArbiterNode::on_tick, this));

    RCLCPP_INFO(get_logger(),
                "mode_arbiter up: %s → %s @ %.1f Hz "
                "(nav=%s approach=%s rail=%s)",
                zone_topic.c_str(), cmd_out.c_str(), rate,
                cmd_nav.c_str(), cmd_approach.c_str(), cmd_rail.c_str());
  }

 private:
  // JSON value extraction helpers. Minimal — no strict parsing. Good
  // enough for the well-formed JSON that zone_detector emits.
  static std::string extract_string(const std::string &d,
                                    const std::string &key) {
    const std::string needle = "\"" + key + "\":\"";
    const auto pos = d.find(needle);
    if (pos == std::string::npos) return "";
    const size_t p = pos + needle.size();
    const auto end = d.find('"', p);
    if (end == std::string::npos) return "";
    return d.substr(p, end - p);
  }

  static double extract_double_or_nan(const std::string &d,
                                       const std::string &key) {
    const std::string needle = "\"" + key + "\":";
    const auto pos = d.find(needle);
    if (pos == std::string::npos) return std::nan("");
    size_t p = pos + needle.size();
    if (p < d.size() && d[p] == 'n') return std::nan("");  // null
    try { return std::stod(d.substr(p)); } catch (...) { return std::nan(""); }
  }

  static int extract_int_or_minus1(const std::string &d,
                                    const std::string &key) {
    const std::string needle = "\"" + key + "\":";
    const auto pos = d.find(needle);
    if (pos == std::string::npos) return -1;
    size_t p = pos + needle.size();
    if (p < d.size() && d[p] == 'n') return -1;  // null
    try { return std::stoi(d.substr(p)); } catch (...) { return -1; }
  }

  void on_zone(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.zone = extract_string(msg->data, "zone");
    latest_inputs_.approach_tag_id =
        extract_int_or_minus1(msg->data, "approach_tag_id");
    latest_inputs_.aisle_y_center =
        extract_double_or_nan(msg->data, "aisle_y_center");
  }

  void on_approach_state(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.rail_approach_state = extract_string(msg->data, "state");
  }

  void on_driver_state(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.rail_driver_state = extract_string(msg->data, "state");
  }

  void on_tick() {
    refresh_exit_clearance_input();

    auto out = step(mode_, latest_inputs_);

    // One-shot side-effects from transitions.
    if (out.request_rail_approach) {
      call_rail_approach_service();
    }
    if (out.request_rail_drive_goal) {
      publish_rail_drive_goal();
    }
    if (out.request_rail_exit_push) {
      publish_rail_exit_push_goal();
    }

    // Reset the in-flight latch when the FSM leaves the approach flow.
    if (out.next_mode == Mode::CORRIDOR_NAV) {
      latest_inputs_.approach_request_in_flight = false;
      have_rail_entry_ = false;
    }

    if (out.next_mode != mode_) {
      // Cache the entry position at any transition *into* RAIL_DRIVE. The
      // arbiter-driven path (RAIL_APPROACH_ACTIVE → RAIL_DRIVE) already did
      // this via publish_rail_drive_goal(); this branch covers the
      // direct-dispatch shortcut (Stage G tests publish straight to
      // /agv/rail_driver/goal). rail_entry_x_ drives both the exit-push
      // goal and the clearance calculation.
      if (out.next_mode == Mode::RAIL_DRIVE && !have_rail_entry_ &&
          !std::isnan(latest_inputs_.current_x)) {
        rail_entry_x_ = latest_inputs_.current_x;
        exit_direction_ = 0.0;  // resolved at exit-push time from travelled dx
        have_rail_entry_ = true;
      }
      RCLCPP_INFO(get_logger(), "mode %s → %s (source=%s)",
                  mode_to_str(mode_), mode_to_str(out.next_mode),
                  source_to_str(out.active_source));
      mode_ = out.next_mode;
      ++transition_count_;
    }
    active_source_ = out.active_source;

    // Relay the selected Twist, or publish zero.
    geometry_msgs::msg::Twist cmd;
    switch (active_source_) {
      case Source::NAV:
        if (last_nav_) cmd = *last_nav_;
        break;
      case Source::APPROACH:
        if (last_approach_) cmd = *last_approach_;
        break;
      case Source::RAIL:
        if (last_rail_) cmd = *last_rail_;
        break;
      case Source::NONE:
      default:
        // Leave cmd zero-initialised.
        break;
    }
    pub_cmd_->publish(cmd);

    // Publish /agv/mode/state.
    std::ostringstream os;
    os << "{\"mode\":\""   << mode_to_str(mode_) << "\","
       << "\"source\":\"" << source_to_str(active_source_) << "\","
       << "\"zone\":\""   << latest_inputs_.zone << "\","
       << "\"operator_mode\":\"" << latest_inputs_.operator_mode << "\","
       << "\"transitions\":" << transition_count_ << "}";
    std_msgs::msg::String state_msg;
    state_msg.data = os.str();
    pub_state_->publish(state_msg);
  }

  void call_rail_approach_service() {
    if (latest_inputs_.approach_tag_id < 0) {
      RCLCPP_WARN(get_logger(),
                  "request_rail_approach fired but approach_tag_id is -1 "
                  "(zone=%s). Holding.",
                  latest_inputs_.zone.c_str());
      return;
    }
    if (!approach_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
                  "rail_approach service not ready; skipping tick.");
      return;
    }
    auto req = std::make_shared<agv_interfaces::srv::RailApproach::Request>();
    req->tag_id = latest_inputs_.approach_tag_id;
    req->offset_x = approach_offset_x_;
    req->offset_y = approach_offset_y_;
    RCLCPP_INFO(get_logger(),
                "→ /agv/rail_approach/execute tag_id=%d offset=(%.2f, %.2f)",
                req->tag_id, req->offset_x, req->offset_y);
    approach_client_->async_send_request(req);
    latest_inputs_.approach_request_in_flight = true;
  }

  void publish_rail_drive_goal() {
    if (std::isnan(latest_inputs_.current_x) ||
        std::isnan(latest_inputs_.aisle_y_center)) {
      RCLCPP_WARN(get_logger(),
                  "request_rail_drive_goal without pose or aisle_y_center; "
                  "holding.");
      return;
    }
    // Direction: approaching REAR (x=4.0) means the next rail segment is
    // to the west (x decreases); approaching FRONT (x=7.0) means east.
    const double dir =
        (latest_inputs_.zone == "rail_approach_rear") ? -1.0 : +1.0;
    const double goal_x =
        latest_inputs_.current_x + dir * rail_drive_distance_m_;
    const double goal_y = latest_inputs_.aisle_y_center;

    // Cache the entry position + outward direction for the later RAIL_EXIT
    // push. Outward is the sign opposite to the inward rail_drive travel.
    rail_entry_x_ = latest_inputs_.current_x;
    exit_direction_ = -dir;
    have_rail_entry_ = true;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.orientation.w = 1.0;
    pub_rail_goal_->publish(goal);
    RCLCPP_INFO(get_logger(),
                "→ /agv/rail_driver/goal (%.3f, %.3f) from zone=%s "
                "current=(%.3f, %.3f) dir=%+.0f",
                goal_x, goal_y, latest_inputs_.zone.c_str(),
                latest_inputs_.current_x, latest_inputs_.current_y, dir);
  }

  void publish_rail_exit_push_goal() {
    if (!have_rail_entry_) {
      RCLCPP_WARN(get_logger(),
                  "request_rail_exit_push without cached entry; holding.");
      return;
    }
    // Direction: outward = opposite of the direction we travelled inward.
    // Resolved now (vs at RAIL_DRIVE entry) because the shortcut path has
    // no a-priori goal to infer from — the robot has to have moved for us
    // to know which way was "in".
    if (exit_direction_ == 0.0) {
      const double dx = latest_inputs_.current_x - rail_entry_x_;
      exit_direction_ = (dx >= 0.0) ? -1.0 : +1.0;  // outward = back toward entry
    }
    const double goal_y = std::isnan(latest_inputs_.aisle_y_center)
                              ? latest_inputs_.current_y
                              : latest_inputs_.aisle_y_center;
    const double goal_x = rail_entry_x_ + exit_direction_ * rail_exit_push_m_;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.orientation.w = 1.0;
    pub_rail_goal_->publish(goal);
    RCLCPP_INFO(get_logger(),
                "→ /agv/rail_driver/goal EXIT_PUSH (%.3f, %.3f) "
                "entry_x=%.3f out_dir=%+.0f",
                goal_x, goal_y, rail_entry_x_, exit_direction_);
  }

  void refresh_exit_clearance_input() {
    if (!have_rail_entry_) {
      latest_inputs_.rail_exit_clearance_m = 0.0;
      return;
    }
    // If exit_direction is not yet resolved (direct-dispatch case), derive
    // it from the travelled dx so far. Same logic as in the push-goal
    // publisher; keeping both in sync lets the FSM see meaningful clearance
    // values before the push goal fires.
    double dir = exit_direction_;
    if (dir == 0.0) {
      const double dx = latest_inputs_.current_x - rail_entry_x_;
      dir = (dx >= 0.0) ? -1.0 : +1.0;
    }
    latest_inputs_.rail_exit_clearance_m =
        dir * (latest_inputs_.current_x - rail_entry_x_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_rail_goal_;
  rclcpp::Client<agv_interfaces::srv::RailApproach>::SharedPtr  approach_client_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_approach_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_rail_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   sub_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_zone_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_approach_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_driver_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_collision_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_operator_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  geometry_msgs::msg::Twist::ConstSharedPtr last_nav_;
  geometry_msgs::msg::Twist::ConstSharedPtr last_approach_;
  geometry_msgs::msg::Twist::ConstSharedPtr last_rail_;

  Mode   mode_ = Mode::CORRIDOR_NAV;
  Source active_source_ = Source::NAV;
  FsmInputs latest_inputs_;
  size_t transition_count_ = 0;

  double rail_drive_distance_m_ = 3.0;
  double rail_exit_push_m_ = 1.5;
  double approach_offset_x_ = 0.3;
  double approach_offset_y_ = 0.0;
  bool auto_approach_ = false;

  // Entry pose cached at RAIL_APPROACH_ACTIVE → RAIL_DRIVE transition.
  // Consumed by the RAIL_EXIT push goal and clearance computation.
  double rail_entry_x_ = 0.0;
  double exit_direction_ = 1.0;
  bool have_rail_entry_ = false;
};

}  // namespace agv_mode_arbiter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_mode_arbiter::ModeArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
