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
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

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
    declare_parameter<double>("publish_rate_hz", 20.0);

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
    const double rate           = get_parameter("publish_rate_hz").as_double();

    pub_cmd_   = create_publisher<geometry_msgs::msg::Twist>(cmd_out, rclcpp::QoS{10});
    pub_state_ = create_publisher<std_msgs::msg::String>(state_topic, rclcpp::QoS{10});

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

  void on_zone(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.zone = extract_string(msg->data, "zone");
  }

  void on_approach_state(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.rail_approach_state = extract_string(msg->data, "state");
  }

  void on_driver_state(std_msgs::msg::String::ConstSharedPtr msg) {
    latest_inputs_.rail_driver_state = extract_string(msg->data, "state");
  }

  void on_tick() {
    auto out = step(mode_, latest_inputs_);

    // One-shot side-effects from transitions.
    if (out.request_rail_approach) {
      RCLCPP_INFO(get_logger(),
                  "FSM requests rail_approach service call (zone=%s)",
                  latest_inputs_.zone.c_str());
      latest_inputs_.approach_request_in_flight = true;
      // Actual service call wired in P2.S4 integration.
    }
    if (out.request_rail_drive_goal) {
      RCLCPP_INFO(get_logger(),
                  "FSM requests rail_driver goal publication (zone=%s)",
                  latest_inputs_.zone.c_str());
      // Actual goal publisher wired in P2.S4 integration.
    }

    // Reset the in-flight latch when the FSM leaves the approach flow.
    if (out.next_mode == Mode::CORRIDOR_NAV) {
      latest_inputs_.approach_request_in_flight = false;
    }

    if (out.next_mode != mode_) {
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

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_approach_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_rail_;
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
};

}  // namespace agv_mode_arbiter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_mode_arbiter::ModeArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
