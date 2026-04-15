#include "agv_safety/safety_supervisor.hpp"

#include <algorithm>
#include <sstream>

namespace agv_safety {

SupervisorVerdict evaluate_topics(
    const std::vector<MonitoredTopic>& topics,
    const rclcpp::Time& now,
    const rclcpp::Time& started_at,
    bool software_estop_latched,
    std::chrono::milliseconds startup_grace) {
  SupervisorVerdict v;

  if (software_estop_latched) {
    v.safety_ok = false;
    v.alerts.push_back("software_estop_latched");
    v.reason = "Software E-stop latched";
  }

  const int64_t since_start_ns = (now - started_at).nanoseconds();
  const int64_t grace_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(startup_grace).count();
  const bool in_grace = since_start_ns < grace_ns;

  for (const auto& t : topics) {
    if (!t.ever_seen) {
      // During startup grace, missing topics are tolerated; afterwards they
      // count as silent.
      if (in_grace) {
        continue;
      }
      v.safety_ok = false;
      v.silent_topics.push_back(t.name);
      continue;
    }
    const auto age_ns = (now - t.last_seen).nanoseconds();
    const auto deadline_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.deadline).count();
    if (age_ns > deadline_ns) {
      v.safety_ok = false;
      v.silent_topics.push_back(t.name);
    }
  }

  if (!v.silent_topics.empty()) {
    std::ostringstream oss;
    oss << "silent: ";
    for (size_t i = 0; i < v.silent_topics.size(); ++i) {
      if (i > 0) oss << ", ";
      oss << v.silent_topics[i];
    }
    v.alerts.push_back(oss.str());
    if (v.reason.empty()) {
      v.reason = oss.str();
    }
  }

  return v;
}

SafetySupervisorNode::SafetySupervisorNode() : rclcpp::Node("safety_supervisor") {
  declare_parameters();
  load_monitored_topics();

  started_at_ = now();
  startup_grace_ = std::chrono::milliseconds(get_parameter("startup_grace_ms").as_int());
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  pub_status_ = create_publisher<agv_interfaces::msg::SafetyStatus>(
      "~/status", rclcpp::QoS(10).reliable());

  sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "software_estop", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&SafetySupervisorNode::on_software_estop, this, std::placeholders::_1));

  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / publish_rate_hz_));
  timer_ = create_wall_timer(period, std::bind(&SafetySupervisorNode::on_tick, this));

  RCLCPP_INFO(get_logger(),
              "safety_supervisor up: %zu monitored topics, %.1f Hz, grace %ldms",
              topics_.size(), publish_rate_hz_,
              static_cast<long>(startup_grace_.count()));
}

void SafetySupervisorNode::declare_parameters() {
  declare_parameter<double>("publish_rate_hz", 10.0);
  declare_parameter<int>("startup_grace_ms", 2000);
  declare_parameter("monitored_topics", std::vector<std::string>{});
  declare_parameter("monitored_types", std::vector<std::string>{});
  declare_parameter("monitored_deadline_ms", std::vector<int64_t>{});
}

void SafetySupervisorNode::load_monitored_topics() {
  const auto names = get_parameter("monitored_topics").as_string_array();
  const auto types = get_parameter("monitored_types").as_string_array();
  const auto deadlines = get_parameter("monitored_deadline_ms").as_integer_array();

  if (names.size() != types.size() || names.size() != deadlines.size()) {
    RCLCPP_ERROR(get_logger(),
                 "monitored_topics/types/deadline_ms must be the same length "
                 "(%zu/%zu/%zu)",
                 names.size(), types.size(), deadlines.size());
    return;
  }

  for (size_t i = 0; i < names.size(); ++i) {
    MonitoredTopic t;
    t.name = names[i];
    t.type = types[i];
    t.deadline = std::chrono::milliseconds(deadlines[i]);
    t.last_seen = now();
    t.ever_seen = false;

    try {
      t.sub = create_generic_subscription(
          t.name, t.type, rclcpp::QoS(10).best_effort(),
          [this, idx = i](std::shared_ptr<const rclcpp::SerializedMessage>) {
            if (idx < topics_.size()) {
              topics_[idx].last_seen = now();
              topics_[idx].ever_seen = true;
            }
          });
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
                  "failed to subscribe to %s (type %s): %s",
                  t.name.c_str(), t.type.c_str(), e.what());
    }

    topics_.push_back(std::move(t));
  }
}

void SafetySupervisorNode::on_tick() {
  const auto verdict = evaluate_topics(
      topics_, now(), started_at_, software_estop_latched_, startup_grace_);

  agv_interfaces::msg::SafetyStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "base_link";
  msg.safety_ok = verdict.safety_ok;
  msg.software_estop = software_estop_latched_;
  msg.silent_topics = verdict.silent_topics;
  msg.alerts = verdict.alerts;
  msg.reason = verdict.reason;
  pub_status_->publish(msg);
}

void SafetySupervisorNode::on_software_estop(const std_msgs::msg::Bool& msg) {
  if (msg.data && !software_estop_latched_) {
    RCLCPP_WARN(get_logger(), "software E-stop latched");
  } else if (!msg.data && software_estop_latched_) {
    RCLCPP_INFO(get_logger(), "software E-stop cleared");
  }
  software_estop_latched_ = msg.data;
}

}  // namespace agv_safety
