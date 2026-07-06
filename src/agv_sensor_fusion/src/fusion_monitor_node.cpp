/**
 * Fusion Monitor Node
 *
 * 1. Subscribes to global EKF output (nav_msgs/Odometry)
 * 2. Republishes as /agv/pose (PoseWithCovarianceStamped) at 10Hz — spec requirement
 * 3. Monitors EKF covariance for localization degradation
 * 4. Publishes diagnostic_msgs/DiagnosticArray with localization health
 * 5. Tracks per-sensor health: wheel odom, cuVSLAM, IMU, AprilTag markers (M6)
 * 6. Subscribes to cuVSLAM tracking state and publishes tracking status (M8)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <deque>
#include <array>
#include <vector>

struct SensorHealth {
  std::string name;
  double expected_hz;
  double staleness_factor;  // stale if no msg for expected_period * factor
  rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> timestamps;  // rolling window for rate calc
  double window_seconds{5.0};
  uint64_t total_count{0};

  double rate() const {
    if (timestamps.size() < 2) return 0.0;
    double span = (timestamps.back() - timestamps.front()).seconds();
    if (span <= 0.0) return 0.0;
    return static_cast<double>(timestamps.size() - 1) / span;
  }

  void record(rclcpp::Time now) {
    last_time = now;
    timestamps.push_back(now);
    total_count++;
    // Prune old entries outside rolling window
    while (!timestamps.empty() &&
           (now - timestamps.front()).seconds() > window_seconds) {
      timestamps.pop_front();
    }
  }

  double age(rclcpp::Time now) const {
    if (total_count == 0) return 999.0;
    return (now - last_time).seconds();
  }

  bool is_stale(rclcpp::Time now) const {
    double period = 1.0 / expected_hz;
    return age(now) > period * staleness_factor;
  }

  bool is_rate_low(rclcpp::Time /*now*/) const {
    return total_count > 2 && rate() < expected_hz * 0.5;
  }
};

class FusionMonitorNode : public rclcpp::Node
{
public:
  FusionMonitorNode() : Node("fusion_monitor")
  {
    // Parameters
    this->declare_parameter("pose_rate_hz", 10.0);
    this->declare_parameter("covariance_warn_threshold", 0.5);
    this->declare_parameter("covariance_error_threshold", 2.0);
    this->declare_parameter("stale_timeout_s", 2.0);
    this->declare_parameter("cuvslam_status_topic",
      std::string("/visual_slam/status"));
    this->declare_parameter("cuvslam_lost_noise_multiplier", 3.0);
    this->declare_parameter("cuvslam_degraded_noise_multiplier", 1.5);
    this->declare_parameter("cuvslam_recovery_delay_s", 2.0);
    this->declare_parameter("cuvslam_cov_degraded_threshold", 0.1);
    this->declare_parameter("cuvslam_cov_critical_threshold", 0.5);
    this->declare_parameter("ekf_global_node_name",
      std::string("ekf_global"));

    pose_rate_hz_ = this->get_parameter("pose_rate_hz").as_double();
    cov_warn_ = this->get_parameter("covariance_warn_threshold").as_double();
    cov_error_ = this->get_parameter("covariance_error_threshold").as_double();
    stale_timeout_ = this->get_parameter("stale_timeout_s").as_double();
    auto cuvslam_topic = this->get_parameter("cuvslam_status_topic").as_string();
    lost_noise_mult_ = this->get_parameter("cuvslam_lost_noise_multiplier").as_double();
    degraded_noise_mult_ = this->get_parameter("cuvslam_degraded_noise_multiplier").as_double();
    recovery_delay_ = this->get_parameter("cuvslam_recovery_delay_s").as_double();
    cov_degraded_thresh_ = this->get_parameter("cuvslam_cov_degraded_threshold").as_double();
    cov_critical_thresh_ = this->get_parameter("cuvslam_cov_critical_threshold").as_double();
    auto ekf_global_name = this->get_parameter("ekf_global_node_name").as_string();

    // Initialize per-sensor health trackers (M6)
    sensor_wheel_odom_ = {"wheel_odom", 50.0, 2.0, {0, 0, RCL_ROS_TIME}, {}, 5.0, 0};
    sensor_cuvslam_ = {"cuVSLAM", 10.0, 2.0, {0, 0, RCL_ROS_TIME}, {}, 5.0, 0};
    sensor_imu_ = {"IMU", 200.0, 2.0, {0, 0, RCL_ROS_TIME}, {}, 5.0, 0};
    sensor_markers_ = {"AprilTag", 0.0, 0.0, {0, 0, RCL_ROS_TIME}, {}, 5.0, 0};  // event-driven

    // ── EKF parameter client for adaptive process noise ──
    ekf_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, ekf_global_name);

    // ── Subscribers ──

    // EKF outputs (existing)
    sub_global_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/global", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_global_odom(msg); });

    sub_local_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/local", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) { last_local_time_ = this->now(); });

    // Per-sensor subscriptions (M6)
    sub_wheel_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv/wheel_odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
        sensor_wheel_odom_.record(this->now());
      });

    sub_cuvslam_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        sensor_cuvslam_.record(this->now());
        // Extract cuVSLAM pose covariance for quality gating
        double max_cov = 0.0;
        for (int i : {0, 7, 35}) {  // x, y, yaw diagonal
          max_cov = std::max(max_cov, std::abs(msg->pose.covariance[i]));
        }
        cuvslam_max_cov_ = max_cov;
        update_cuvslam_quality_level();
      });

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/agv/zed/imu/data", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr /*msg*/) {
        sensor_imu_.record(this->now());
      });

    sub_marker_pose_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/agv/marker_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
        sensor_markers_.record(this->now());
      });

    // cuVSLAM tracking state (M8)
    // The exact topic depends on isaac_ros_visual_slam version.
    // Common options: /visual_slam/status, /visual_slam/tracking/vo_pose_covariance
    sub_cuvslam_status_ = this->create_subscription<std_msgs::msg::String>(
      cuvslam_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_cuvslam_status(msg);
      });

    // ── Publishers ──

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", rclcpp::QoS(10));

    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    // cuVSLAM tracking OK boolean (M8) — downstream nodes can subscribe
    pub_cuvslam_ok_ = this->create_publisher<std_msgs::msg::Bool>(
      "/agv/cuvslam_tracking_ok", rclcpp::QoS(10));

    // Timer for pose publishing at configured rate
    auto period = std::chrono::duration<double>(1.0 / pose_rate_hz_);
    timer_ = this->create_wall_timer(period, [this]() { publish_pose(); });

    // Diagnostic timer at 1Hz
    diag_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_diagnostics(); });

    RCLCPP_INFO(this->get_logger(),
      "Fusion monitor: pose at %.0fHz, cov warn=%.2f error=%.2f, "
      "per-sensor health enabled, cuVSLAM status topic: %s",
      pose_rate_hz_, cov_warn_, cov_error_, cuvslam_topic.c_str());
  }

private:
  void on_global_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_global_odom_ = *msg;
    last_global_time_ = this->now();
    has_global_ = true;

    // Extract max diagonal covariance for health monitoring
    max_pose_cov_ = 0.0;
    for (int i : {0, 7, 35}) {  // x, y, yaw diagonal
      max_pose_cov_ = std::max(max_pose_cov_, std::abs(msg->pose.covariance[i]));
    }
  }

  void on_cuvslam_status(const std_msgs::msg::String::SharedPtr msg)
  {
    // cuVSLAM status strings vary by version. Common values:
    //   "TRACKING", "LOST", "INITIALIZING", "SUCCESS"
    std::string status = msg->data;
    bool was_ok = cuvslam_tracking_ok_;

    cuvslam_tracking_ok_ =
      (status.find("TRACKING") != std::string::npos) ||
      (status.find("SUCCESS") != std::string::npos);

    if (!cuvslam_tracking_ok_ && was_ok) {
      RCLCPP_WARN(this->get_logger(),
        "cuVSLAM tracking lost — inflating global EKF process noise x%.1f",
        lost_noise_mult_);
      apply_noise_multiplier(lost_noise_mult_);
    } else if (cuvslam_tracking_ok_ && !was_ok) {
      RCLCPP_INFO(this->get_logger(),
        "cuVSLAM tracking recovered — restoring EKF noise after %.1fs delay",
        recovery_delay_);
      // Schedule noise restoration after recovery delay
      recovery_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(recovery_delay_),
        [this]() {
          if (cuvslam_tracking_ok_ && cuvslam_quality_level_ == QualityLevel::TRACKING) {
            apply_noise_multiplier(1.0);
            RCLCPP_INFO(this->get_logger(), "Global EKF process noise restored to baseline");
          }
          recovery_timer_->cancel();
        });
    }

    last_cuvslam_status_ = status;

    // Publish boolean for downstream nodes
    std_msgs::msg::Bool ok_msg;
    ok_msg.data = cuvslam_tracking_ok_;
    pub_cuvslam_ok_->publish(ok_msg);
  }

  // 3-level quality assessment based on cuVSLAM covariance
  enum class QualityLevel { TRACKING, DEGRADED, LOST };

  void update_cuvslam_quality_level()
  {
    QualityLevel prev = cuvslam_quality_level_;
    if (!cuvslam_tracking_ok_) {
      cuvslam_quality_level_ = QualityLevel::LOST;
    } else if (cuvslam_max_cov_ > cov_critical_thresh_) {
      cuvslam_quality_level_ = QualityLevel::LOST;
    } else if (cuvslam_max_cov_ > cov_degraded_thresh_) {
      cuvslam_quality_level_ = QualityLevel::DEGRADED;
    } else {
      cuvslam_quality_level_ = QualityLevel::TRACKING;
    }

    // Apply appropriate noise multiplier on transitions
    if (cuvslam_quality_level_ != prev) {
      switch (cuvslam_quality_level_) {
        case QualityLevel::DEGRADED:
          RCLCPP_WARN(this->get_logger(),
            "cuVSLAM quality DEGRADED (cov=%.3f) — inflating EKF noise x%.1f",
            cuvslam_max_cov_, degraded_noise_mult_);
          apply_noise_multiplier(degraded_noise_mult_);
          break;
        case QualityLevel::LOST:
          // Only apply if status-based handler hasn't already applied lost multiplier
          if (prev == QualityLevel::DEGRADED) {
            RCLCPP_WARN(this->get_logger(),
              "cuVSLAM quality CRITICAL (cov=%.3f) — inflating EKF noise x%.1f",
              cuvslam_max_cov_, lost_noise_mult_);
            apply_noise_multiplier(lost_noise_mult_);
          }
          break;
        case QualityLevel::TRACKING:
          if (prev == QualityLevel::DEGRADED) {
            RCLCPP_INFO(this->get_logger(), "cuVSLAM quality restored to TRACKING");
            apply_noise_multiplier(1.0);
          }
          break;
      }
    }
  }

  void apply_noise_multiplier(double multiplier)
  {
    desired_noise_mult_ = multiplier;

    if (!ekf_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "EKF global parameter service not ready — cannot adjust process noise");
      return;
    }

    // Capture baseline on first use — fully async. This method runs inside
    // subscription/timer callbacks of a single-threaded executor: the
    // parameter response can only be delivered by that same executor thread,
    // so any blocking wait/get here would deadlock the node permanently.
    if (!has_baseline_noise_) {
      if (baseline_request_pending_) {
        // Request in flight; its callback re-applies on arrival. Re-arm if
        // the response never came (e.g. ekf_global restarted mid-request).
        if ((this->now() - baseline_request_time_).seconds() < 10.0) {
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "Baseline process-noise request unanswered for 10s — retrying");
      }
      baseline_request_pending_ = true;
      baseline_request_time_ = this->now();
      ekf_param_client_->get_parameters(
        {"process_noise_covariance"},
        [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
          baseline_request_pending_ = false;
          try {
            auto results = future.get();
            if (results.empty()) {
              RCLCPP_WARN(this->get_logger(),
                "EKF global returned no process_noise_covariance — "
                "adaptive noise disabled");
              return;
            }
            baseline_process_noise_ = results[0].as_double_array();
            has_baseline_noise_ = true;
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(),
              "Failed to get baseline process noise: %s", e.what());
            return;
          }
          // Apply the most recently requested multiplier (quality may have
          // transitioned again while the request was in flight).
          apply_noise_multiplier(desired_noise_mult_);
        });
      return;
    }

    // Apply multiplier to diagonal entries only
    std::vector<double> adjusted = baseline_process_noise_;
    for (size_t i = 0; i < 15 && i * 15 + i < adjusted.size(); ++i) {
      adjusted[i * 15 + i] *= multiplier;
    }

    auto param = rclcpp::Parameter("process_noise_covariance", adjusted);
    ekf_param_client_->set_parameters({param});
    current_noise_mult_ = multiplier;
  }

  void publish_pose()
  {
    if (!has_global_) return;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = last_global_odom_.header;
    pose_msg.pose = last_global_odom_.pose;
    pub_pose_->publish(pose_msg);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->now();

    auto now = this->now();

    // ── Overall localization status (existing) ──
    {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "fusion_monitor: Localization";
      status.hardware_id = "agv_sensor_fusion";

      double global_age = has_global_ ? (now - last_global_time_).seconds() : 999.0;
      double local_age = (now - last_local_time_).seconds();

      if (global_age > stale_timeout_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Global EKF output stale";
      } else if (local_age > stale_timeout_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Local EKF output stale";
      } else if (max_pose_cov_ > cov_error_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Localization covariance critically high";
      } else if (max_pose_cov_ > cov_warn_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Localization covariance elevated";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Localization healthy";
      }

      auto kv = [](const std::string& k, double v) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = k;
        entry.value = std::to_string(v);
        return entry;
      };

      status.values.push_back(kv("max_pose_covariance", max_pose_cov_));
      status.values.push_back(kv("global_ekf_age_s", global_age));
      status.values.push_back(kv("local_ekf_age_s", local_age));

      diag_array.status.push_back(status);
    }

    // ── Per-sensor health diagnostics (M6) ──
    auto add_sensor_diag = [&](SensorHealth& sensor) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "fusion_monitor: Sensor/" + sensor.name;
      status.hardware_id = "agv_sensor_fusion";

      auto kv_str = [](const std::string& k, const std::string& v) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = k;
        entry.value = v;
        return entry;
      };

      double current_rate = sensor.rate();
      double age = sensor.age(now);

      status.values.push_back(kv_str("rate_hz",
        std::to_string(current_rate)));
      status.values.push_back(kv_str("expected_hz",
        std::to_string(sensor.expected_hz)));
      status.values.push_back(kv_str("age_s",
        std::to_string(age)));
      status.values.push_back(kv_str("total_messages",
        std::to_string(sensor.total_count)));

      // Event-driven sensors (markers) have no rate expectation
      if (sensor.expected_hz <= 0.0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = sensor.total_count > 0
          ? "Detections received (event-driven)"
          : "No detections yet (event-driven)";
      } else if (sensor.is_stale(now)) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Stale — no messages for " +
          std::to_string(age) + "s";
        RCLCPP_WARN(this->get_logger(),
          "%s STALE: no messages for %.1fs (expected %.0f Hz)",
          sensor.name.c_str(), age, sensor.expected_hz);
      } else if (sensor.is_rate_low(now)) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Rate low: " +
          std::to_string(current_rate) + " Hz (expected " +
          std::to_string(sensor.expected_hz) + " Hz)";
        RCLCPP_WARN(this->get_logger(),
          "%s rate low: %.1f Hz (expected %.0f Hz)",
          sensor.name.c_str(), current_rate, sensor.expected_hz);
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "OK: " +
          std::to_string(current_rate) + " Hz";
      }

      diag_array.status.push_back(status);
    };

    add_sensor_diag(sensor_wheel_odom_);
    add_sensor_diag(sensor_cuvslam_);
    add_sensor_diag(sensor_imu_);
    add_sensor_diag(sensor_markers_);

    // ── cuVSLAM tracking state (M8) ──
    {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "fusion_monitor: cuVSLAM Tracking";
      status.hardware_id = "agv_sensor_fusion";

      diagnostic_msgs::msg::KeyValue state_kv;
      state_kv.key = "tracking_state";
      state_kv.value = last_cuvslam_status_.empty() ? "unknown" : last_cuvslam_status_;
      status.values.push_back(state_kv);

      diagnostic_msgs::msg::KeyValue ok_kv;
      ok_kv.key = "tracking_ok";
      ok_kv.value = cuvslam_tracking_ok_ ? "true" : "false";
      status.values.push_back(ok_kv);

      diagnostic_msgs::msg::KeyValue quality_kv;
      quality_kv.key = "quality_level";
      quality_kv.value = (cuvslam_quality_level_ == QualityLevel::TRACKING) ? "TRACKING" :
                         (cuvslam_quality_level_ == QualityLevel::DEGRADED) ? "DEGRADED" : "LOST";
      status.values.push_back(quality_kv);

      diagnostic_msgs::msg::KeyValue cov_kv;
      cov_kv.key = "max_covariance";
      cov_kv.value = std::to_string(cuvslam_max_cov_);
      status.values.push_back(cov_kv);

      diagnostic_msgs::msg::KeyValue mult_kv;
      mult_kv.key = "noise_multiplier";
      mult_kv.value = std::to_string(current_noise_mult_);
      status.values.push_back(mult_kv);

      if (last_cuvslam_status_.empty()) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "No cuVSLAM status received yet";
      } else if (cuvslam_quality_level_ == QualityLevel::LOST) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "cuVSLAM LOST: " + last_cuvslam_status_;
      } else if (cuvslam_quality_level_ == QualityLevel::DEGRADED) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "cuVSLAM DEGRADED (cov=" + std::to_string(cuvslam_max_cov_) + ")";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "cuVSLAM tracking OK";
      }

      diag_array.status.push_back(status);
    }

    pub_diag_->publish(diag_array);
  }

  // Subscribers — EKF outputs
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_local_;

  // Subscribers — individual sensors (M6)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_cuvslam_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_marker_pose_;

  // Subscriber — cuVSLAM status (M8)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cuvslam_status_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cuvslam_ok_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // State
  nav_msgs::msg::Odometry last_global_odom_;
  rclcpp::Time last_global_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_time_{0, 0, RCL_ROS_TIME};
  bool has_global_{false};
  double max_pose_cov_{0.0};

  // Per-sensor health (M6)
  SensorHealth sensor_wheel_odom_;
  SensorHealth sensor_cuvslam_;
  SensorHealth sensor_imu_;
  SensorHealth sensor_markers_;

  // cuVSLAM tracking state (M8)
  bool cuvslam_tracking_ok_{true};  // assume OK until told otherwise
  std::string last_cuvslam_status_;

  // cuVSLAM quality gating
  double cuvslam_max_cov_{0.0};
  QualityLevel cuvslam_quality_level_{QualityLevel::TRACKING};

  // Adaptive EKF process noise
  std::shared_ptr<rclcpp::AsyncParametersClient> ekf_param_client_;
  std::vector<double> baseline_process_noise_;
  bool has_baseline_noise_{false};
  bool baseline_request_pending_{false};
  rclcpp::Time baseline_request_time_{0, 0, RCL_ROS_TIME};
  double current_noise_mult_{1.0};
  double desired_noise_mult_{1.0};
  rclcpp::TimerBase::SharedPtr recovery_timer_;

  // Parameters
  double pose_rate_hz_;
  double cov_warn_;
  double cov_error_;
  double stale_timeout_;
  double lost_noise_mult_;
  double degraded_noise_mult_;
  double recovery_delay_;
  double cov_degraded_thresh_;
  double cov_critical_thresh_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
