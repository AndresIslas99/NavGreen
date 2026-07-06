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
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_interfaces/srv/rail_approach.hpp"

#include "agv_mode_arbiter/mode_fsm.hpp"
#include "agv_mode_arbiter/rail_exit_geometry.hpp"

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
    // String side-channel ("stop"/"slowdown"/"clear") — published in HIL by
    // agv_hil_bridges/sim_obstacle_relay. Nav2's real collision_monitor does
    // NOT publish this type; see collision_monitor_state_topic below.
    declare_parameter<std::string>("collision_topic",
                                    "/agv/collision_monitor_state");
    // Typed production safety source: Nav2's collision_monitor publishes
    // nav2_msgs/CollisionMonitorState here. Set to "" to disable.
    declare_parameter<std::string>("collision_monitor_state_topic",
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
    // Greenhouse-aisle geometry used by the RAIL_EXIT push-goal and
    // clearance logic. Defaults match zone_classifier_impl.hpp; override
    // per-deployment in mode_arbiter_params.yaml. The arbiter picks the
    // outward direction from `current_x` vs the gap bounds (no travel-
    // history heuristic), so shortcut-dispatched rail_drive exits work
    // regardless of which side the goal was on.
    declare_parameter<double>("rail_exit_rear_tag_x",  4.0);
    declare_parameter<double>("rail_exit_front_tag_x", 7.0);
    declare_parameter<double>("rail_exit_gap_x_min",   3.5);
    declare_parameter<double>("rail_exit_gap_x_max",   7.5);
    // Camera-to-tag desired forward offset when requesting rail_approach.
    declare_parameter<double>("approach_offset_x", 0.3);
    declare_parameter<double>("approach_offset_y", 0.0);
    // Gate: if false, the arbiter never auto-calls rail_approach on zone
    // entry. It still relays the right cmd_vel source when rail_approach
    // is triggered externally (operator, test harness). Default false so
    // the test_waypoint_precision Nav2-direct path keeps working.
    declare_parameter<bool>("auto_approach", false);
    // Iter-22 brain 1.2 — FSM anti-oscillation guards.
    // Round-44 iter-20/21 brain_log showed the arbiter bouncing
    // rail_exit ↔ corridor_nav ↔ rail_drive 4+ times in 300 ms during
    // RAIL_EXIT push; at 10 Hz harness polling the corridor_nav tick was
    // frequently missed, yielding NAV_TIMEOUT on wp13/wp14 even though
    // the geometric release actually fired.
    //   - min_mode_dwell_s: enforce minimum dwell time between mode
    //     transitions. Safety stops bypass the gate (hard priority).
    //   - push_sticky: publish the rail_exit push goal ONCE per RAIL_EXIT
    //     entry, not every tick the FSM happens to set
    //     request_rail_exit_push. Resets on leaving RAIL_EXIT.
    // Iter-22b: default 0.0 (disabled) after iter-22 observed wp12
    // regression (tag lost during fine_servoing) and wp15 regression
    // (err grew from 2.16 to 3.52 m). The dwell was suppressing
    // legitimate fast transitions like CORRIDOR_NAV → RAIL_APPROACH_
    // ACTIVE when rail_approach state went driving in <0.5 s after
    // a prior mode change. Set to 0.0 so the gate is a no-op; the
    // sticky push flag below still prevents double-publication of
    // EXIT_PUSH without blocking any transitions.
    declare_parameter<double>("min_mode_dwell_s", 0.0);

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
    const auto collision_state_topic =
        get_parameter("collision_monitor_state_topic").as_string();
    const auto operator_topic   = get_parameter("operator_mode_topic").as_string();
    const auto state_topic      = get_parameter("state_topic").as_string();
    const auto odom_topic       = get_parameter("odom_topic").as_string();
    const auto rail_goal_topic  = get_parameter("rail_goal_topic").as_string();
    const auto rail_approach_service =
        get_parameter("rail_approach_service").as_string();
    const double rate           = get_parameter("publish_rate_hz").as_double();
    rail_drive_distance_m_ = get_parameter("rail_drive_distance_m").as_double();
    geom_.push_m     = get_parameter("rail_exit_push_m").as_double();
    geom_.tag_x_rear = get_parameter("rail_exit_rear_tag_x").as_double();
    geom_.tag_x_front = get_parameter("rail_exit_front_tag_x").as_double();
    geom_.gap_x_min  = get_parameter("rail_exit_gap_x_min").as_double();
    geom_.gap_x_max  = get_parameter("rail_exit_gap_x_max").as_double();
    approach_offset_x_ = get_parameter("approach_offset_x").as_double();
    approach_offset_y_ = get_parameter("approach_offset_y").as_double();
    auto_approach_ = get_parameter("auto_approach").as_bool();
    latest_inputs_.auto_approach = auto_approach_;
    min_mode_dwell_s_ = get_parameter("min_mode_dwell_s").as_double();

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
          safety_stop_string_ = (msg->data.find("stop") != std::string::npos);
        });
    // Nav2's collision_monitor publishes the typed CollisionMonitorState —
    // a DDS type distinct from the String side-channel above, so the String
    // subscription never matches it on the real robot. Subscribe to both and
    // OR them each tick (either source saying STOP forces BLOCKED_HANDOFF).
    if (!collision_state_topic.empty()) {
      sub_collision_state_ =
          create_subscription<nav2_msgs::msg::CollisionMonitorState>(
              collision_state_topic, rclcpp::QoS{10},
              [this](nav2_msgs::msg::CollisionMonitorState::ConstSharedPtr msg) {
                safety_stop_nav2_ =
                    (msg->action_type ==
                     nav2_msgs::msg::CollisionMonitorState::STOP);
              });
    }
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
    // Merge the two collision sources (String side-channel + typed Nav2
    // state). Conservative OR: safety_stop clears only when both are clear.
    latest_inputs_.safety_stop = safety_stop_string_ || safety_stop_nav2_;

    // Refresh RAIL_EXIT clearance from geometry (aisle-side, not cached
    // entry): outward distance from the nearest approach tag. Positive
    // past the tag. The FSM's release gate reads this on the next step.
    if (!std::isnan(latest_inputs_.current_x)) {
      const auto g = compute_rail_exit(latest_inputs_.current_x, geom_);
      latest_inputs_.rail_exit_clearance_m = g.clearance_m;
    } else {
      latest_inputs_.rail_exit_clearance_m = 0.0;
    }

    auto out = step(mode_, latest_inputs_);

    // Iter-22 brain 1.2 — anti-oscillation dwell gate. If the FSM wants
    // to change mode AND the safety chain is NOT overriding (BLOCKED_
    // HANDOFF always passes), require at least `min_mode_dwell_s_`
    // since the last actual change. Otherwise suppress the change and
    // keep the current mode + its source. Prevents the
    // rail_exit ↔ rail_drive ↔ corridor_nav rapid-fire seen in iter-20/21
    // during RAIL_EXIT push which lost the corridor_nav tick in harness
    // polling.
    const auto now_ns = this->now().nanoseconds();
    const bool is_safety_override = (out.next_mode == Mode::BLOCKED_HANDOFF);
    const double since_last_change_s =
        (last_mode_change_ns_ == 0)
            ? std::numeric_limits<double>::infinity()
            : (now_ns - last_mode_change_ns_) / 1e9;
    bool dwell_blocked = false;
    if (out.next_mode != mode_ && !is_safety_override &&
        since_last_change_s < min_mode_dwell_s_) {
      // Hold current mode. Don't propagate the new source either — keep
      // relaying whatever the current mode implies.
      dwell_blocked = true;
      out.next_mode = mode_;
      out.active_source = active_source_;
      // Drop one-shot requests too — they would fire on the transition
      // that we just denied.
      out.request_rail_approach = false;
      out.request_rail_drive_goal = false;
      out.request_rail_exit_push = false;
    }

    // One-shot side-effects from transitions (after dwell gate).
    if (out.request_rail_approach) {
      call_rail_approach_service();
    }
    if (out.request_rail_drive_goal) {
      publish_rail_drive_goal();
    }
    // Iter-22 brain 1.2 — push sticky: emit the exit push goal ONCE per
    // RAIL_EXIT entry instead of every tick the FSM asserts
    // request_rail_exit_push. Reset when leaving RAIL_EXIT. Prevents the
    // double-publish observed iter-20/21 where the robot got a second
    // EXIT_PUSH mid-traversal (`clearance_now=-2.95` = robot 2.95 m
    // past the tag after the first push started but a second push
    // re-fired).
    if (out.request_rail_exit_push && !push_published_this_exit_) {
      publish_rail_exit_push_goal();
      push_published_this_exit_ = true;
    }
    if (mode_ == Mode::RAIL_EXIT && out.next_mode != Mode::RAIL_EXIT) {
      // Leaving RAIL_EXIT — arm the flag for the next entry.
      push_published_this_exit_ = false;
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
      last_mode_change_ns_ = now_ns;
      ++transition_count_;
    } else if (dwell_blocked) {
      RCLCPP_DEBUG(get_logger(),
                   "mode dwell: suppressed change (<%.2f s since last)",
                   min_mode_dwell_s_);
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

    // Iter-33 diagnostics: RAIL source is wz=0 by contract. If we ever
    // publish a non-zero angular.z, something upstream is mis-routed
    // (stale cmd_vel_approach latch, NAV source leaking past FSM, or
    // the Twist mutated between subscription and relay). iter-32
    // c5_drive_in had the robot rotate 64° during RAIL source — this
    // throttled warn will name which source carried the rotation.
    if (std::abs(cmd.angular.z) > 0.05) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
          "cmd_vel relay: wz=%.3f under source=%s (RAIL must be 0). "
          "last_nav wz=%.3f  last_approach wz=%.3f  last_rail wz=%.3f",
          cmd.angular.z, source_to_str(active_source_),
          last_nav_ ? last_nav_->angular.z : 0.0,
          last_approach_ ? last_approach_->angular.z : 0.0,
          last_rail_ ? last_rail_->angular.z : 0.0);
    }

    // Phase-2 teleop bypass: when the operator owns the joystick, teleop_server
    // publishes to /agv/cmd_vel directly. If the arbiter also publishes a zero
    // Twist at 20 Hz, the two streams alternate on the topic, the velocity
    // smoother averages them toward zero, and the deadband (0.01 m/s) collapses
    // the output — the robot sits still even with the stick pinned. Keep the
    // /agv/mode/state broadcast below so dashboards still see the current mode.
    const bool teleop_owned = (latest_inputs_.operator_mode == "teleop" &&
                               active_source_ == Source::NONE);
    if (!teleop_owned) {
      pub_cmd_->publish(cmd);
    }

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
    if (std::isnan(latest_inputs_.current_x)) {
      RCLCPP_WARN(get_logger(),
                  "request_rail_exit_push without pose; holding.");
      return;
    }
    const auto g = compute_rail_exit(latest_inputs_.current_x, geom_);
    if (g.skip_push) {
      // Already ≥ 1 m past the exit tag, or sitting in the gap with
      // no clear inward direction. The FSM release gate will fire on
      // the next tick; no need for a new goal.
      RCLCPP_INFO(get_logger(),
                  "rail_exit_push skipped: clearance=%.2f m "
                  "(current_x=%.3f)", g.clearance_m,
                  latest_inputs_.current_x);
      return;
    }
    const double goal_y = std::isnan(latest_inputs_.aisle_y_center)
                              ? latest_inputs_.current_y
                              : latest_inputs_.aisle_y_center;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = get_clock()->now();
    goal.header.frame_id = "map";
    goal.pose.position.x = g.push_goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.orientation.w = 1.0;
    pub_rail_goal_->publish(goal);
    RCLCPP_INFO(get_logger(),
                "→ /agv/rail_driver/goal EXIT_PUSH (%.3f, %.3f) "
                "clearance_now=%.2f",
                g.push_goal_x, goal_y, g.clearance_m);
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
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr
      sub_collision_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_operator_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  geometry_msgs::msg::Twist::ConstSharedPtr last_nav_;
  geometry_msgs::msg::Twist::ConstSharedPtr last_approach_;
  geometry_msgs::msg::Twist::ConstSharedPtr last_rail_;

  Mode   mode_ = Mode::CORRIDOR_NAV;
  Source active_source_ = Source::NAV;
  FsmInputs latest_inputs_;
  // Last-seen stop flag per collision source; OR-ed into
  // latest_inputs_.safety_stop at the top of every tick.
  bool safety_stop_string_ = false;
  bool safety_stop_nav2_ = false;
  size_t transition_count_ = 0;
  // Iter-22 brain 1.2 — anti-oscillation state.
  int64_t last_mode_change_ns_ = 0;       // 0 = no changes yet.
  bool push_published_this_exit_ = false;  // one-shot per RAIL_EXIT entry.

  double rail_drive_distance_m_ = 3.0;
  double approach_offset_x_ = 0.3;
  double approach_offset_y_ = 0.0;
  bool auto_approach_ = false;
  double min_mode_dwell_s_ = 0.5;

  // Greenhouse geometry for RAIL_EXIT push-goal + clearance. Populated
  // from parameters at ctor; the push logic is stateless (depends only
  // on current_x + these constants), which lets the arbiter exit rails
  // correctly regardless of which side the rail_drive goal was on.
  RailExitGeometryParams geom_;
};

}  // namespace agv_mode_arbiter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_mode_arbiter::ModeArbiterNode>());
  rclcpp::shutdown();
  return 0;
}
