// auto_init_orchestrator_node — automatic localization coordinator for the
// AGV greenhouse.
//
// Hierarchy (evaluated in order on every map load):
//   1. AprilTag pose hint — preferred because it's absolute ground truth,
//      robust to lighting, independent of visual drift.
//   2. Last-known pose from disk ({map_name}_meta.json) — second best, uses
//      the pose at the time the map was last saved or the robot last shut
//      down cleanly.
//   3. FAILED state — the dashboard surfaces this only as an informational
//      LOC pill. Recovery path: operator drives via teleop toward an
//      AprilTag and calls /agv/localization/reinitialize.
//
// Published state topic: /agv/localization/state (std_msgs/String with a
// simple "INITIALIZING" | "LOCALIZED" | "DEGRADED" | "FAILED" payload). The
// backend mirrors this for dashboard display; nav goals are NOT gated on it.
//
// Inputs:
//   - /agv/maps/loaded (std_msgs/String) — map name without extension.
//     Fired by map_manager_node after a successful load_map.
//   - /agv/marker_pose (geometry_msgs/PoseWithCovarianceStamped) — AprilTag
//     pose from marker_correction_node.
//
// Services called:
//   - /visual_slam/load_map (isaac_ros_visual_slam_interfaces/FilePath)
//   - /visual_slam/localize_in_map (isaac_ros_visual_slam_interfaces/LocalizeInMap)
//
// Services exposed:
//   - localization/reinitialize (std_srvs/Trigger) — force a re-init pass
//     against the currently loaded map (operator button in dashboard).
//
// NOT a lifecycle node — the state is held in memory, and a crash takes the
// whole localization out anyway (no use in deferring start-up).

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/file_path.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/localize_in_map.hpp>
#include <robot_localization/srv/set_pose.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {
constexpr const char* STATE_INITIALIZING = "INITIALIZING";
constexpr const char* STATE_LOCALIZED    = "LOCALIZED";
constexpr const char* STATE_DEGRADED     = "DEGRADED";
constexpr const char* STATE_FAILED       = "FAILED";

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

Pose2D pose_from_quat(double qz, double qw, double x, double y) {
  Pose2D p;
  p.x = x;
  p.y = y;
  // yaw from z+w components of a 2D quaternion (roll=pitch=0)
  p.theta = 2.0 * std::atan2(qz, qw);
  return p;
}
}  // namespace

class AutoInitOrchestratorNode : public rclcpp::Node {
public:
  AutoInitOrchestratorNode() : Node("auto_init_orchestrator") {
    this->declare_parameter("map_dir", "");
    this->declare_parameter("marker_wait_timeout_s", 10.0);
    this->declare_parameter("localize_retries", 3);
    this->declare_parameter("localize_retry_backoff_s", 2.0);
    this->declare_parameter("last_known_pose_filename_suffix", "_meta.json");
    this->declare_parameter("kidnapping_drift_m", 5.0);
    // Periodic persistence: save the current pose to disk every N seconds so
    // that a hard crash (power loss, OOM kill) still leaves a recent
    // last-known-pose for the next boot. Set to 0 to disable periodic saves;
    // the shutdown hook still runs on SIGINT/SIGTERM.
    this->declare_parameter("periodic_save_interval_s", 30.0);

    map_dir_ = this->get_parameter("map_dir").as_string();
    marker_timeout_s_ = this->get_parameter("marker_wait_timeout_s").as_double();
    localize_retries_ = this->get_parameter("localize_retries").as_int();
    retry_backoff_s_ = this->get_parameter("localize_retry_backoff_s").as_double();
    meta_suffix_ = this->get_parameter("last_known_pose_filename_suffix").as_string();
    periodic_save_interval_s_ = this->get_parameter("periodic_save_interval_s").as_double();

    if (map_dir_.empty()) {
      RCLCPP_FATAL(get_logger(), "map_dir parameter is required");
      throw std::runtime_error("map_dir not set");
    }

    // Service clients for cuVSLAM (absolute topics, not in our namespace)
    load_map_client_ = this->create_client<
      isaac_ros_visual_slam_interfaces::srv::FilePath>(
        "/visual_slam/load_map");
    localize_client_ = this->create_client<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>(
        "/visual_slam/localize_in_map");
    // robot_localization set_pose — used by the Path-B fallback when there is
    // no cuVSLAM keyframe DB for the current map. Lets the orchestrator drive
    // ekf_global directly from an AprilTag pose or a last-known-pose JSON.
    set_pose_client_ = this->create_client<robot_localization::srv::SetPose>(
      "set_pose");

    // State publisher with transient_local durability so late-joining clients
    // (like the dashboard reconnecting) get the current state immediately.
    rclcpp::QoS state_qos(1);
    state_qos.transient_local().reliable();
    state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "localization/state", state_qos);

    // Subscribe to map_manager's maps/loaded event (transient_local so we get
    // the most recently loaded map even if we started late).
    rclcpp::QoS tl_qos(1);
    tl_qos.transient_local().reliable();
    map_loaded_sub_ = this->create_subscription<std_msgs::msg::String>(
      "maps/loaded", tl_qos,
      std::bind(&AutoInitOrchestratorNode::on_map_loaded, this, std::placeholders::_1));

    // AprilTag pose — best-effort QoS to match marker_correction_node's publisher
    marker_pose_sub_ = this->create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
        "marker_pose", rclcpp::QoS(10),
        std::bind(&AutoInitOrchestratorNode::on_marker_pose, this,
                  std::placeholders::_1));

    // Re-initialize trigger (operator button)
    reinit_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "localization/reinitialize",
      std::bind(&AutoInitOrchestratorNode::on_reinitialize, this,
                std::placeholders::_1, std::placeholders::_2));

    // Track the live pose so we can persist it to disk. The orchestrator does
    // not consume the pose for its own logic (cuVSLAM does that); it's only
    // for the last-known-pose fallback file.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/global", rclcpp::QoS(10),
      std::bind(&AutoInitOrchestratorNode::on_odometry, this, std::placeholders::_1));

    // Periodic save timer. Saves only when a map is loaded AND a fresh pose
    // has been received. Cheap — atomic snapshot + small JSON file write.
    if (periodic_save_interval_s_ > 0.0) {
      const auto period = std::chrono::milliseconds(
        static_cast<int64_t>(periodic_save_interval_s_ * 1000));
      save_timer_ = this->create_wall_timer(period, [this]() {
        save_last_known_pose_to_disk("periodic");
      });
    }

    // Graceful-shutdown hook: capture the final pose on SIGINT/SIGTERM from
    // `ros2 launch` so the next boot can fall back to it if no AprilTag is
    // visible. This runs before rclcpp context shutdown tears down the node.
    rclcpp::on_shutdown([this]() {
      save_last_known_pose_to_disk("shutdown");
    });

    set_state(STATE_FAILED, "boot — no map loaded yet");
    RCLCPP_INFO(get_logger(), "auto_init_orchestrator ready, map_dir=%s",
                map_dir_.c_str());
  }

private:
  // ── Event handlers ──────────────────────────────────────────────────────

  void on_map_loaded(const std_msgs::msg::String::SharedPtr msg) {
    const std::string map_name = msg->data;
    RCLCPP_INFO(get_logger(), "maps/loaded received: '%s' — starting auto-init",
                map_name.c_str());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_map_name_ = map_name;
    }
    // Run the sequence in a short-lived worker thread so we don't block the
    // executor callback loop (the marker-wait + retries can take seconds).
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread(&AutoInitOrchestratorNode::run_init_sequence, this, map_name);
  }

  void on_marker_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(marker_mutex_);
    last_marker_pose_ = *msg;
    last_marker_time_ = this->now();
    has_marker_ = true;
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const auto& p = msg->pose.pose;
    last_odom_pose_ = pose_from_quat(p.orientation.z, p.orientation.w,
                                     p.position.x, p.position.y);
    has_odom_ = true;
  }

  void on_reinitialize(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::string map_name;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name = current_map_name_;
    }
    if (map_name.empty()) {
      res->success = false;
      res->message = "No map loaded — nothing to initialize against";
      return;
    }
    RCLCPP_INFO(get_logger(), "Manual reinitialize requested for map '%s'",
                map_name.c_str());
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread(&AutoInitOrchestratorNode::run_init_sequence,
                          this, map_name);
    res->success = true;
    res->message = "Reinitialization started";
  }

  // ── Sequence ────────────────────────────────────────────────────────────

  // Cascading localization strategy — the orchestrator must NEVER ask the
  // operator to manually pose-click unless EVERY automatic path has been
  // exhausted. The cascade is:
  //
  //   Path A (preferred): cuVSLAM visual relocalization against a saved
  //     keyframe database + pose hint. Best quality, continuous tracking.
  //
  //   Path B (legacy maps): AprilTag absolute reference. Works when the
  //     map has no cuVSLAM DB (old maps, maps from a different robot) as
  //     long as an AprilTag is in view. Uses marker_correction_node's
  //     pose directly to seed ekf_global via set_pose.
  //
  //   Path C (stale-pose fallback): last-known-pose JSON from the previous
  //     session. Only usable if the robot has not been physically moved
  //     since shutdown. Degraded state — operator should drive slowly.
  //
  //   FAILED is reached only when A, B and C all fail. In that case the
  //   dashboard shows the manual pose modal as an absolute last resort.
  void run_init_sequence(const std::string& map_name) {
    set_state(STATE_INITIALIZING, "selecting localization strategy");

    const std::string cuvslam_folder = map_dir_ + "/" + map_name + "_cuvslam";
    const bool has_cuvslam_db = fs::exists(cuvslam_folder);

    if (has_cuvslam_db) {
      set_state(STATE_INITIALIZING, "Path A: loading cuVSLAM keyframe DB");
      if (try_cuvslam_path(cuvslam_folder, map_name)) {
        return;  // LOCALIZED or DEGRADED state already published
      }
      RCLCPP_WARN(get_logger(),
        "Path A (cuVSLAM) failed, falling through to Path B");
    } else {
      RCLCPP_INFO(get_logger(),
        "No cuVSLAM keyframe DB at %s — skipping Path A, trying AprilTag fallback",
        cuvslam_folder.c_str());
    }

    // Path B + C: wait for any pose hint (AprilTag first, last-known second)
    set_state(STATE_INITIALIZING,
              has_cuvslam_db ? "Path B: visual DB failed, awaiting AprilTag or last-known"
                             : "Path B: no visual DB, awaiting AprilTag or last-known");
    auto hint = wait_for_pose_hint(map_name);
    if (!hint.has_value()) {
      set_state(STATE_FAILED,
                "no visual DB, no AprilTag, no last-known pose — manual pose required");
      return;
    }

    const std::string& hint_source = hint->first;
    const Pose2D& pose = hint->second;
    RCLCPP_INFO(get_logger(),
      "Path B/C pose hint from %s: (%.2f, %.2f, %.2f°)",
      hint_source.c_str(), pose.x, pose.y, pose.theta * 180.0 / M_PI);

    // Drive ekf_global directly via set_pose. This is the same mechanism the
    // manual modal uses — we just call it from the orchestrator automatically.
    if (!call_set_pose(pose)) {
      set_state(STATE_FAILED,
                "set_pose service call failed — ekf_global may be down");
      return;
    }

    if (hint_source == "apriltag") {
      set_state(STATE_LOCALIZED,
                "Path B: AprilTag absolute anchor (no visual keyframe DB for this map)");
    } else {
      // last_known — marked DEGRADED because we have no active confirmation
      // that the robot is still at the stored pose. marker_correction_node
      // will upgrade to LOCALIZED state once it sees a tag.
      set_state(STATE_DEGRADED,
                "Path C: last-known pose from previous session (drive slowly)");
    }
  }

  // Tries the cuVSLAM path: load_map → wait pose hint → localize_in_map
  //                        → set_pose on ekf_global with the same hint.
  //
  // Why set_pose is mandatory even in Path A:
  //   ekf_global consumes /visual_slam/tracking/odometry in DIFFERENTIAL
  //   mode (see ekf_global.yaml). Differential mode uses velocity deltas
  //   only, NOT the absolute pose cuVSLAM reports after localize_in_map.
  //   Without an absolute seed, ekf_global stays near its boot origin
  //   (0,0,0) and integrates visual deltas from there — its map→odom TF
  //   would diverge from the real robot location in the map frame.
  //
  //   Seeding ekf_global with the same hint pose used for localize_in_map
  //   aligns the global filter with the loaded map immediately. cuVSLAM
  //   refines the estimate over the next few seconds via its differential
  //   deltas, and marker_pose absolute corrections keep it honest.
  //
  // Returns true if successfully localized, false on any failure (caller
  // then falls through to Path B).
  bool try_cuvslam_path(const std::string& cuvslam_folder,
                        const std::string& map_name) {
    if (!call_load_map(cuvslam_folder)) {
      return false;
    }
    set_state(STATE_INITIALIZING, "Path A: waiting for pose hint");
    auto hint = wait_for_pose_hint(map_name);
    if (!hint.has_value()) {
      return false;
    }
    const std::string hint_source = hint->first;
    const Pose2D& pose = hint->second;
    RCLCPP_INFO(get_logger(),
      "Path A pose hint from %s: (%.2f, %.2f, %.2f°)",
      hint_source.c_str(), pose.x, pose.y, pose.theta * 180.0 / M_PI);

    set_state(STATE_INITIALIZING, "Path A: calling localize_in_map");
    for (int attempt = 1; attempt <= localize_retries_ && rclcpp::ok(); ++attempt) {
      if (call_localize_in_map(cuvslam_folder, pose)) {
        // Seed ekf_global with the hint pose so its map→odom TF snaps to
        // the loaded map immediately. Without this, cuVSLAM's differential
        // deltas alone would not move ekf_global off (0,0,0).
        if (!call_set_pose(pose)) {
          RCLCPP_ERROR(get_logger(),
            "Path A: localize_in_map ok but set_pose to ekf_global failed");
          return false;
        }
        if (hint_source == "apriltag") {
          set_state(STATE_LOCALIZED,
                    "cuVSLAM relocalized with AprilTag anchor");
        } else {
          set_state(STATE_DEGRADED,
                    "cuVSLAM relocalized from last-known pose — awaiting AprilTag confirmation");
        }
        return true;
      }
      RCLCPP_WARN(get_logger(),
        "localize_in_map attempt %d/%d failed, retrying in %.1fs",
        attempt, localize_retries_, retry_backoff_s_);
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(retry_backoff_s_ * 1000)));
    }
    return false;
  }

  // Wait up to marker_timeout_s_ for an AprilTag. Fall back to last-known-pose
  // on disk. Returns {source, pose} or nullopt if neither available.
  std::optional<std::pair<std::string, Pose2D>> wait_for_pose_hint(
    const std::string& map_name)
  {
    // 2a. Drain any stale marker from before the map load
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      has_marker_ = false;
    }

    const auto deadline = this->now() + rclcpp::Duration::from_seconds(marker_timeout_s_);
    while (rclcpp::ok() && this->now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        if (has_marker_) {
          const auto& p = last_marker_pose_.pose.pose;
          return std::make_pair<std::string, Pose2D>(
            "apriltag",
            pose_from_quat(p.orientation.z, p.orientation.w,
                           p.position.x, p.position.y));
        }
      }
      std::this_thread::sleep_for(200ms);
    }

    // 2b. Fall back to disk
    const auto meta_path = map_dir_ + "/" + map_name + meta_suffix_;
    if (!fs::exists(meta_path)) {
      RCLCPP_WARN(get_logger(),
        "No AprilTag within %.1fs and no last-known-pose at %s",
        marker_timeout_s_, meta_path.c_str());
      return std::nullopt;
    }
    try {
      std::ifstream in(meta_path);
      std::string content((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
      // Naive JSON parse — format is strictly controlled: {"x":..,"y":..,"theta":..,"saved_at":..}
      Pose2D p;
      p.x = extract_number(content, "\"x\"");
      p.y = extract_number(content, "\"y\"");
      p.theta = extract_number(content, "\"theta\"");
      return std::make_pair<std::string, Pose2D>("last_known", std::move(p));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s", meta_path.c_str(), e.what());
      return std::nullopt;
    }
  }

  double extract_number(const std::string& json, const std::string& key) {
    auto k = json.find(key);
    if (k == std::string::npos) throw std::runtime_error("missing key " + key);
    auto colon = json.find(':', k);
    if (colon == std::string::npos) throw std::runtime_error("bad json near " + key);
    // Skip whitespace
    size_t i = colon + 1;
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t')) ++i;
    size_t end = i;
    while (end < json.size() &&
           (std::isdigit(json[end]) || json[end] == '.' ||
            json[end] == '-' || json[end] == '+' ||
            json[end] == 'e' || json[end] == 'E')) ++end;
    return std::stod(json.substr(i, end - i));
  }

  // ── Service calls ───────────────────────────────────────────────────────

  bool call_load_map(const std::string& cuvslam_folder) {
    if (!load_map_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/load_map not available");
      return false;
    }
    auto req = std::make_shared<
      isaac_ros_visual_slam_interfaces::srv::FilePath::Request>();
    req->file_path = cuvslam_folder;
    auto future = load_map_client_->async_send_request(req);
    if (future.wait_for(15s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/load_map timed out");
      return false;
    }
    try {
      auto r = future.get();
      if (!r->success) {
        RCLCPP_ERROR(get_logger(), "/visual_slam/load_map returned failure");
        return false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/load_map exception: %s", e.what());
      return false;
    }
    return true;
  }

  // Directly drive ekf_global to a specific 2D pose via robot_localization's
  // SetPose service. Used by Path B (AprilTag fallback) and Path C (last-known
  // pose fallback) when cuVSLAM is not available or fails. Covariance is tight
  // because the caller has an authoritative hint.
  bool call_set_pose(const Pose2D& hint) {
    if (!set_pose_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "set_pose service not available");
      return false;
    }
    auto req = std::make_shared<robot_localization::srv::SetPose::Request>();
    req->pose.header.frame_id = "map";
    req->pose.header.stamp = this->now();
    req->pose.pose.pose.position.x = hint.x;
    req->pose.pose.pose.position.y = hint.y;
    req->pose.pose.pose.position.z = 0.0;
    req->pose.pose.pose.orientation.x = 0.0;
    req->pose.pose.pose.orientation.y = 0.0;
    req->pose.pose.pose.orientation.z = std::sin(hint.theta / 2.0);
    req->pose.pose.pose.orientation.w = std::cos(hint.theta / 2.0);
    // 6x6 row-major covariance: tight on x,y,yaw; locked on z,roll,pitch (2D).
    for (int i = 0; i < 36; ++i) req->pose.pose.covariance[i] = 0.0;
    req->pose.pose.covariance[0]  = 0.05;   // x variance (m²)
    req->pose.pose.covariance[7]  = 0.05;   // y variance
    req->pose.pose.covariance[14] = 1e6;    // z — locked
    req->pose.pose.covariance[21] = 1e6;    // roll — locked
    req->pose.pose.covariance[28] = 1e6;    // pitch — locked
    req->pose.pose.covariance[35] = 0.10;   // yaw variance (rad²)

    auto future = set_pose_client_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "set_pose service call timed out");
      return false;
    }
    try {
      future.get();  // robot_localization SetPose has no success field; no exception = OK
      RCLCPP_INFO(get_logger(),
        "set_pose applied: (%.2f, %.2f, %.1f°)",
        hint.x, hint.y, hint.theta * 180.0 / M_PI);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "set_pose exception: %s", e.what());
      return false;
    }
  }

  bool call_localize_in_map(const std::string& cuvslam_folder, const Pose2D& hint) {
    if (!localize_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/localize_in_map not available");
      return false;
    }
    auto req = std::make_shared<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Request>();
    req->map_folder_path = cuvslam_folder;
    req->pose_hint.position.x = hint.x;
    req->pose_hint.position.y = hint.y;
    req->pose_hint.position.z = 0.0;
    req->pose_hint.orientation.x = 0.0;
    req->pose_hint.orientation.y = 0.0;
    req->pose_hint.orientation.z = std::sin(hint.theta / 2.0);
    req->pose_hint.orientation.w = std::cos(hint.theta / 2.0);
    auto future = localize_client_->async_send_request(req);
    if (future.wait_for(30s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/localize_in_map timed out");
      return false;
    }
    try {
      auto r = future.get();
      return r->success;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "/visual_slam/localize_in_map exception: %s", e.what());
      return false;
    }
  }

  // ── Last-known-pose persistence ────────────────────────────────────────

  // Write {map}_meta.json with the current pose. Called periodically (so hard
  // crashes still leave a recent snapshot) and once on graceful shutdown
  // (SIGINT/SIGTERM from `ros2 launch`). Safe to call repeatedly — atomic
  // write via temp file + rename to avoid half-written files if we're killed
  // mid-write.
  void save_last_known_pose_to_disk(const std::string& reason) {
    std::string map_name;
    {
      // Read current_map_name_ (written from the worker thread; we use a
      // shared_mutex-style guard but a simple lock is fine given the low
      // contention on this path).
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name = current_map_name_;
    }
    if (map_name.empty()) return;  // no map loaded → nothing meaningful to save

    Pose2D pose;
    bool have_pose = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (has_odom_) {
        pose = last_odom_pose_;
        have_pose = true;
      }
    }
    if (!have_pose) return;  // no odometry received yet

    const auto final_path = map_dir_ + "/" + map_name + meta_suffix_;
    const auto tmp_path = final_path + ".tmp";
    try {
      std::ofstream out(tmp_path, std::ios::trunc);
      if (!out.is_open()) {
        RCLCPP_WARN(get_logger(), "Could not open %s for writing", tmp_path.c_str());
        return;
      }
      const auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      out << "{\"x\": " << pose.x
          << ", \"y\": " << pose.y
          << ", \"theta\": " << pose.theta
          << ", \"saved_at\": " << now_sec
          << ", \"reason\": \"" << reason << "\"}\n";
      out.close();
      // Atomic rename so readers never see a half-written file
      fs::rename(tmp_path, final_path);
      if (reason == "shutdown") {
        RCLCPP_INFO(get_logger(),
          "Last-known-pose saved (%s): map='%s' pose=(%.3f, %.3f, %.1f°) → %s",
          reason.c_str(), map_name.c_str(), pose.x, pose.y,
          pose.theta * 180.0 / M_PI, final_path.c_str());
      } else {
        RCLCPP_DEBUG(get_logger(), "Periodic last-known-pose save: %s", final_path.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to save last-known-pose: %s", e.what());
    }
  }

  // ── State publication ──────────────────────────────────────────────────

  void set_state(const std::string& action, const std::string& detail) {
    std::string map_snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state_ = action;
      map_snapshot = current_map_name_;
    }
    std_msgs::msg::String msg;
    // JSON payload: easier for the TypeScript backend than a custom msg
    msg.data = "{\"action\":\"" + action + "\",\"detail\":\"" + detail +
               "\",\"map\":\"" + map_snapshot + "\"}";
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "STATE: %s — %s", action.c_str(), detail.c_str());
  }

  // ── Members ────────────────────────────────────────────────────────────

  std::string map_dir_;
  double marker_timeout_s_{10.0};
  int localize_retries_{3};
  double retry_backoff_s_{2.0};
  std::string meta_suffix_{"_meta.json"};
  double periodic_save_interval_s_{30.0};

  std::mutex state_mutex_;  // protects current_map_name_ + current_state_
  std::string current_map_name_;
  std::string current_state_;
  std::thread worker_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_loaded_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr marker_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinit_srv_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr load_map_client_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>::SharedPtr localize_client_;
  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
  rclcpp::TimerBase::SharedPtr save_timer_;

  std::mutex marker_mutex_;
  bool has_marker_{false};
  rclcpp::Time last_marker_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseWithCovarianceStamped last_marker_pose_;

  std::mutex odom_mutex_;
  bool has_odom_{false};
  Pose2D last_odom_pose_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoInitOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
