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
  // Sprint D / HIGH-09-02 (2026-05-13 audit). Per-topic QoS reliability
  // class. Empty array (default) preserves backward compatibility:
  // every monitored topic gets BEST_EFFORT. When provided, this array
  // must be the same length as monitored_topics. Valid values:
  //   - "best_effort"             (default)
  //   - "reliable"
  //   - "reliable_transient_local"
  // Required when a critical topic is published RELIABLE — a
  // best_effort subscriber may still receive it (DDS allows the
  // asymmetry) but if the publisher's durability is transient_local
  // and the subscriber's is volatile, late-joining messages get lost.
  declare_parameter("monitored_qos", std::vector<std::string>{});
}

namespace {
// Build a QoS profile from the canonical name. Unknown names fall back
// to best_effort — the caller logs the warning.
rclcpp::QoS qos_from_name(const std::string& name) {
  if (name == "reliable") {
    return rclcpp::QoS(10).reliable();
  } else if (name == "reliable_transient_local") {
    return rclcpp::QoS(10).reliable().transient_local();
  }
  return rclcpp::QoS(10).best_effort();
}
}  // namespace

void SafetySupervisorNode::load_monitored_topics() {
  const auto names = get_parameter("monitored_topics").as_string_array();
  const auto types = get_parameter("monitored_types").as_string_array();
  const auto deadlines = get_parameter("monitored_deadline_ms").as_integer_array();
  const auto qos_names = get_parameter("monitored_qos").as_string_array();

  if (names.size() != types.size() || names.size() != deadlines.size()) {
    RCLCPP_ERROR(get_logger(),
                 "monitored_topics/types/deadline_ms must be the same length "
                 "(%zu/%zu/%zu)",
                 names.size(), types.size(), deadlines.size());
    return;
  }
  // monitored_qos is optional. If non-empty it must match length;
  // empty preserves the pre-Sprint-D default of best_effort everywhere.
  if (!qos_names.empty() && qos_names.size() != names.size()) {
    RCLCPP_ERROR(get_logger(),
                 "monitored_qos, when provided, must match monitored_topics "
                 "length (got %zu, expected %zu). Falling back to best_effort "
                 "for all topics.",
                 qos_names.size(), names.size());
  }
  const bool use_per_topic_qos =
      !qos_names.empty() && qos_names.size() == names.size();

  for (size_t i = 0; i < names.size(); ++i) {
    MonitoredTopic t;
    t.name = names[i];
    t.type = types[i];
    t.deadline = std::chrono::milliseconds(deadlines[i]);
    t.last_seen = now();
    t.ever_seen = false;

    // Resolve per-topic QoS or default.
    std::string qos_label = "best_effort";
    rclcpp::QoS qos = rclcpp::QoS(10).best_effort();
    if (use_per_topic_qos) {
      qos_label = qos_names[i];
      qos = qos_from_name(qos_label);
      if (qos_label != "best_effort" && qos_label != "reliable" &&
          qos_label != "reliable_transient_local") {
        RCLCPP_WARN(get_logger(),
                    "monitored_qos[%zu]='%s' for topic '%s' is not one of "
                    "{best_effort, reliable, reliable_transient_local}; "
                    "falling back to best_effort.",
                    i, qos_label.c_str(), t.name.c_str());
        qos_label = "best_effort (fallback)";
        qos = rclcpp::QoS(10).best_effort();
      }
    }

    try {
      t.sub = create_generic_subscription(
          t.name, t.type, qos,
          [this, idx = i](std::shared_ptr<const rclcpp::SerializedMessage>) {
            if (idx < topics_.size()) {
              topics_[idx].last_seen = now();
              topics_[idx].ever_seen = true;
            }
          });
      RCLCPP_INFO(get_logger(),
                  "monitor %s (type %s, deadline %ld ms, qos %s)",
                  t.name.c_str(), t.type.c_str(),
                  static_cast<long>(t.deadline.count()),
                  qos_label.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
                  "failed to subscribe to %s (type %s, qos %s): %s",
                  t.name.c_str(), t.type.c_str(), qos_label.c_str(), e.what());
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
