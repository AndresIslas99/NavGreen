#include "agv_rail_approach/rail_approach_node.hpp"
#include "agv_rail_approach/fine_servo_controller.hpp"

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace agv_rail_approach {

RailApproachNode::RailApproachNode() : Node("rail_approach") {
  // Parameters
  declare_parameter("registry_file", "");
  declare_parameter("runtime_registry_file", "");
  declare_parameter("tag_size", 0.2);
  declare_parameter("coarse_standoff_distance", 0.5);
  declare_parameter("default_offset_x", 0.3);
  declare_parameter("default_offset_y", 0.0);
  declare_parameter("tolerance_xy", 0.003);
  declare_parameter("tolerance_yaw", 0.015);
  // Iter-10 / Option A: the legacy yaw-convergence check assumes a
  // wall tag (local X axis horizontal, face normal pointing at the
  // camera). Floor-plane tags — which every rail_approach target in
  // this project is — give a reference angle of π under the same
  // formula, so `in_tolerance` never latches. Default false so the
  // servo settles on position only; set true per-deployment if
  // re-introducing wall-tag targets.
  declare_parameter("check_yaw_convergence", false);
  declare_parameter("settle_frames", 5);
  declare_parameter("Kp_linear", 0.15);
  declare_parameter("Kp_lateral", 0.3);
  declare_parameter("Kp_yaw", 0.5);
  // Iter-46 Paso 1.b: PI + stiction feedforward on the forward axis.
  // Both default to 0 (pure-P legacy). HIL launch sets Ki_linear=0.05
  // and stiction_ff_vel_mps=0.035 based on empirical plant ID.
  declare_parameter("Ki_linear", 0.0);
  declare_parameter("stiction_ff_vel_mps", 0.0);
  declare_parameter("max_fine_linear_vel", 0.03);
  declare_parameter("max_fine_angular_vel", 0.10);
  declare_parameter("tag_loss_timeout_s", 0.5);
  declare_parameter("tag_reacquire_timeout_s", 3.0);
  declare_parameter("acquisition_timeout_s", 3.0);
  // Iter-13 / Option D: cap how long the fine-servo phase may run
  // before the node declares the approach failed. Without this, a
  // servo that oscillates inside the 2 cm ring but never latches
  // settle_frames_ frames in a row stays in FINE_SERVOING forever —
  // the harness was carrying the real deadline at NAV_TIMEOUT_S × 1.5.
  // An internal budget gives rail_approach the chance to publish
  // `state=aborted` with last_reject_reason='fine_servo_timeout' so
  // operators see the failure specifically, not a generic NAV_TIMEOUT.
  declare_parameter("max_fine_duration_s", 120.0);
  // Iter-15: robot-to-tag distance threshold (m) below which the node
  // skips Nav2 coarse_approach entirely. See start_coarse_approach for
  // rationale (floor tags carry yaw=0, Nav2 would get a wrong-direction
  // goal; spawn distances under 1.5 m are already inside fine_servo's
  // range anyway).
  declare_parameter("coarse_skip_radius", 2.0);
  declare_parameter("camera_frame", "zed_left_camera_optical_frame");
  declare_parameter("base_frame", "base_link");
  declare_parameter("camera_info_topic", "/agv/zed/left/camera_info");
  declare_parameter("detections_topic", "detections");
  // Iter-44 Fase 2 Arch A: registry-aware longitudinal override.
  // See rail_approach_node.hpp header for the rationale. Off by default.
  declare_parameter("use_registry_longitudinal", false);
  declare_parameter("registry_max_stale_s", 2.0);

  registry_file_ = get_parameter("registry_file").as_string();
  runtime_registry_file_ = get_parameter("runtime_registry_file").as_string();
  default_tag_size_ = get_parameter("tag_size").as_double();
  coarse_standoff_ = get_parameter("coarse_standoff_distance").as_double();
  default_offset_x_ = get_parameter("default_offset_x").as_double();
  tolerance_xy_ = get_parameter("tolerance_xy").as_double();
  tolerance_yaw_ = get_parameter("tolerance_yaw").as_double();
  settle_frames_ = get_parameter("settle_frames").as_int();
  kp_linear_ = get_parameter("Kp_linear").as_double();
  kp_lateral_ = get_parameter("Kp_lateral").as_double();
  kp_yaw_ = get_parameter("Kp_yaw").as_double();
  ki_linear_ = get_parameter("Ki_linear").as_double();
  stiction_ff_vel_mps_ = get_parameter("stiction_ff_vel_mps").as_double();
  max_fine_linear_ = get_parameter("max_fine_linear_vel").as_double();
  max_fine_angular_ = get_parameter("max_fine_angular_vel").as_double();
  check_yaw_convergence_ = get_parameter("check_yaw_convergence").as_bool();
  tag_loss_timeout_ = get_parameter("tag_loss_timeout_s").as_double();
  tag_reacquire_timeout_ = get_parameter("tag_reacquire_timeout_s").as_double();
  acquisition_timeout_ = get_parameter("acquisition_timeout_s").as_double();
  max_fine_duration_ = get_parameter("max_fine_duration_s").as_double();
  coarse_skip_radius_ = get_parameter("coarse_skip_radius").as_double();
  camera_frame_ = get_parameter("camera_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  use_registry_longitudinal_ = get_parameter("use_registry_longitudinal").as_bool();
  registry_max_stale_s_ = get_parameter("registry_max_stale_s").as_double();

  auto cam_info_topic = get_parameter("camera_info_topic").as_string();
  auto detections_topic = get_parameter("detections_topic").as_string();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    detections_topic, 10,
    std::bind(&RailApproachNode::on_detection, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cam_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&RailApproachNode::on_camera_info, this, std::placeholders::_1));

  // Publishers
  // The topic is `cmd_vel` so launches can remap it — under the `agv`
  // namespace the default resolves to `/agv/cmd_vel`. The Phase-2 mode
  // arbiter remaps this to `/agv/cmd_vel_approach` so Nav2, rail_driver,
  // and rail_approach can coexist.
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Phase 2 convention: publish /agv/rail_approach/state (matches
  // agv_rail_driver and agv_mode_arbiter). The previous `status` topic
  // name was a pre-arbiter convention.
  status_pub_ = create_publisher<std_msgs::msg::String>("rail_approach/state", 10);
  target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("rail_approach/target_pose", 10);

  // Services
  execute_srv_ = create_service<agv_interfaces::srv::RailApproach>(
    "rail_approach/execute",
    std::bind(&RailApproachNode::on_execute, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  abort_srv_ = create_service<std_srvs::srv::Trigger>(
    "rail_approach/abort",
    std::bind(&RailApproachNode::on_abort, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  list_srv_ = create_service<agv_interfaces::srv::ListRailStarts>(
    "rail_approach/list_rail_starts",
    std::bind(&RailApproachNode::on_list, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavAction>(this, "navigate_to_pose");

  // Status timer — 10 Hz so the mode_arbiter (ticking at 20 Hz) sees
  // transitions within one tick. 2 Hz was too slow for FSM pickup.
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RailApproachNode::publish_status, this));

  // Hot reload trigger from ui_backend (when operator assigns/removes tags)
  auto reload_qos = rclcpp::QoS(1).transient_local().reliable();
  reload_sub_ = create_subscription<std_msgs::msg::Empty>(
    "markers/registry_reload", reload_qos,
    [this](std_msgs::msg::Empty::SharedPtr) {
      RCLCPP_INFO(get_logger(), "Reloading rail starts from disk");
      reload_all_registries();
    });

  // Localization state cache. We extract the "action" string from the
  // JSON payload published by auto_init_orchestrator. Used by the gate
  // in on_execute to reject coarse_approach paths when the EKF anchor
  // is not LOCALIZED.
  // Sprint C / MEDIUM-10-06 (2026-05-13 audit). The publisher
  // (auto_init_orchestrator at "localization/state") uses
  // QoS(1).transient_local().reliable() so late-joining subscribers
  // receive the most recent state on connect. Without transient_local
  // here, rail_approach booting after auto_init has already declared
  // LOCALIZED would never see the latched message — the gate in
  // on_execute would reject coarse_approach paths until the next
  // state transition (which may not come for minutes). Match the
  // publisher's durability.
  loc_state_sub_ = create_subscription<std_msgs::msg::String>(
    "localization/state", rclcpp::QoS(1).transient_local().reliable(),
    [this](std_msgs::msg::String::SharedPtr msg) {
      const std::string& d = msg->data;
      const std::string key = "\"action\":\"";
      auto p = d.find(key);
      if (p == std::string::npos) return;
      p += key.size();
      auto e = d.find('"', p);
      if (e == std::string::npos) return;
      last_localization_action_ = d.substr(p, e - p);
    });

  reload_all_registries();

  RCLCPP_INFO(get_logger(), "Rail approach node ready, %zu rail starts loaded", rail_starts_.size());
}

// Reload from both static (registry_file) and runtime (runtime_registry_file) sources
void RailApproachNode::reload_all_registries() {
  rail_starts_.clear();
  if (!registry_file_.empty()) load_registry_from(registry_file_);
  if (!runtime_registry_file_.empty()) {
    std::ifstream test(runtime_registry_file_);
    if (test.is_open()) {
      test.close();
      load_registry_from(runtime_registry_file_);
    }
  }
}

// Original load_registry() now wraps load_registry_from for backward compatibility
void RailApproachNode::load_registry() {
  if (!registry_file_.empty()) load_registry_from(registry_file_);
}

// ── Registry loading ──

void RailApproachNode::load_registry_from(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_WARN(get_logger(), "Cannot open registry: %s", path.c_str());
    return;
  }

  // Simple YAML parser: find entries with type: rail_start
  // Reads line-by-line looking for marker blocks with type field
  std::string line;
  int current_id = -1;
  double x = 0, y = 0, z = 0, yaw = 0;
  double tag_size = default_tag_size_;
  bool is_rail_start = false;
  bool in_marker = false;

  // Round 44 iter-7: auto-classify floor tags as rail_start when no
  // explicit type is set. markers_registry.yaml historically omitted the
  // `type` field on the floor-plane rail entry tags (z=0.002 m at x=4.0
  // REAR and x=7.0 FRONT for ids 2, 3, 4, 12, 13, 33–37), so every
  // rail_approach service call returned "Unknown rail start tag ID" even
  // though the detections were flowing. Falling back on z-height keeps
  // the registry file as the single source of truth.
  constexpr double FLOOR_TAG_Z_M = 0.05;  // ≤ this counts as floor-plane.

  auto flush_marker = [&]() {
    const bool is_floor = z < FLOOR_TAG_Z_M;
    if (in_marker && (is_rail_start || is_floor) && current_id >= 0) {
      rail_starts_[current_id] = {current_id, x, y, yaw, tag_size};
      RCLCPP_INFO(get_logger(),
                  "Rail start: tag %d at (%.2f, %.2f) yaw=%.2f size=%.3f%s",
                  current_id, x, y, yaw, tag_size,
                  (is_rail_start ? "" : " [auto: floor tag]"));
    }
    current_id = -1;
    x = y = z = yaw = 0;
    tag_size = default_tag_size_;
    is_rail_start = false;
  };

  while (std::getline(file, line)) {
    // Skip comments and empty lines
    auto trimmed = line;
    auto pos = trimmed.find_first_not_of(' ');
    if (pos == std::string::npos || trimmed[pos] == '#') continue;
    trimmed = trimmed.substr(pos);

    // Detect new marker entry
    if (trimmed.find("- id:") == 0) {
      flush_marker();
      in_marker = true;
      try { current_id = std::stoi(trimmed.substr(5)); } catch (...) { current_id = -1; }
      continue;
    }

    if (!in_marker) continue;

    // Parse fields
    auto parse_double = [](const std::string& s, const std::string& key) -> double {
      auto p = s.find(key);
      if (p == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
      try { return std::stod(s.substr(p + key.size())); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
    };

    auto val = parse_double(trimmed, "x:");
    if (!std::isnan(val)) { x = val; continue; }
    val = parse_double(trimmed, "y:");
    if (!std::isnan(val)) { y = val; continue; }
    val = parse_double(trimmed, "z:");
    if (!std::isnan(val)) { z = val; continue; }
    val = parse_double(trimmed, "yaw:");
    if (!std::isnan(val)) { yaw = val; continue; }
    val = parse_double(trimmed, "tag_size:");
    if (!std::isnan(val)) { tag_size = val; continue; }

    if (trimmed.find("type:") != std::string::npos) {
      if (trimmed.find("rail_start") != std::string::npos) {
        is_rail_start = true;
      }
    }
  }
  flush_marker();  // last entry
}

// ── Camera info ──

void RailApproachNode::on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (camera_info_received_) return;
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  if (fx_ > 0 && fy_ > 0) {
    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(), "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
  }
}

// ── Service: execute ──

void RailApproachNode::on_execute(
    const std::shared_ptr<rmw_request_id_t>,
    const agv_interfaces::srv::RailApproach::Request::SharedPtr req,
    agv_interfaces::srv::RailApproach::Response::SharedPtr resp) {

  // Iter-17: IDLE is the pristine ready state, but after a prior run the
  // node now latches SETTLED (success) or ABORTED (failure) so the
  // harness can see the outcome. Both are safe re-entry points for a
  // new approach — only the active-flow states (COARSE_APPROACH,
  // TAG_ACQUISITION, FINE_SERVOING) should reject a new service call.
  if (state_ != State::IDLE &&
      state_ != State::SETTLED &&
      state_ != State::ABORTED) {
    resp->success = false;
    resp->message = "Approach already in progress (state: " + state_name() + ")";
    return;
  }

  auto it = rail_starts_.find(req->tag_id);
  if (it == rail_starts_.end()) {
    resp->success = false;
    resp->message = "Unknown rail start tag ID: " + std::to_string(req->tag_id);
    return;
  }

  if (!camera_info_received_) {
    resp->success = false;
    resp->message = "Camera intrinsics not yet received";
    return;
  }

  // Localization gate. Coarse approach delegates to Nav2, which uses
  // map→base_link TF; if localization is not verified, that TF is
  // stale or biased and the robot drives to a phantom pose (the bug
  // observed 2026-04-25). When skip_coarse_approach=true the path
  // uses direct AprilTag detection only; map state is irrelevant.
  if (!req->skip_coarse_approach && last_localization_action_ != "LOCALIZED") {
    resp->success = false;
    resp->message = "Localization is " + last_localization_action_ +
        "; pass skip_coarse_approach=true for direct fine-servoing, "
        "or reinitialize localization first.";
    return;
  }

  target_tag_id_ = req->tag_id;
  desired_offset_x_ = (req->offset_x > 0.01) ? req->offset_x : default_offset_x_;
  desired_offset_y_ = req->offset_y;
  settle_count_ = 0;

  RCLCPP_INFO(get_logger(),
              "Starting rail approach to tag %d (offset: %.3f, %.3f, skip_coarse=%s)",
              target_tag_id_, desired_offset_x_, desired_offset_y_,
              req->skip_coarse_approach ? "true" : "false");

  if (req->skip_coarse_approach) {
    // Skip Nav2; jump straight to TAG_ACQUISITION. Resets the same
    // PI/median-filter state that start_coarse_approach would clear.
    fine_servo_state_.reset();
    last_fine_servo_tick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    pnp_filter_ = TvecRvecMedianFilter(pnp_filter_window_);
    fine_servo_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_reject_reason_ = "none";
    last_reject_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    state_ = State::TAG_ACQUISITION;
    acquisition_start_ = now();
    RCLCPP_INFO(get_logger(),
                "skip_coarse_approach=true → entering TAG_ACQUISITION directly");
  } else {
    start_coarse_approach(it->second);
  }

  // Iter-17c: publish status IMMEDIATELY so the harness's
  // _wait_for_state_value doesn't latch onto the stale "aborted" /
  // "settled" string from a prior approach. Without this, the next
  // waypoint's rail_approach dispatch can read a state message that
  // predates the new execute() call, fail instantly, and report
  // ABORTED dur=0 before any motion happens (observed on wp07 in
  // iter-17b right after wp04 aborted via the timeout path).
  publish_status();

  // Don't respond yet — response will be sent when approach completes.
  // For now, respond immediately with "in progress" since ROS2 services are synchronous.
  resp->success = true;
  resp->message = "Approach started";
}

// ── Service: abort ──

void RailApproachNode::on_abort(
    const std::shared_ptr<rmw_request_id_t>,
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr resp) {

  if (state_ == State::IDLE) {
    resp->success = true;
    resp->message = "Already idle";
    return;
  }

  RCLCPP_WARN(get_logger(), "Abort requested in state %s", state_name().c_str());
  finish(false, "Aborted by operator");
  resp->success = true;
  resp->message = "Approach aborted";
}

// ── Service: list rail starts ──

void RailApproachNode::on_list(
    const std::shared_ptr<rmw_request_id_t>,
    const agv_interfaces::srv::ListRailStarts::Request::SharedPtr,
    agv_interfaces::srv::ListRailStarts::Response::SharedPtr resp) {

  for (const auto& [id, rs] : rail_starts_) {
    agv_interfaces::msg::RailStartPoint pt;
    pt.tag_id = rs.id;
    pt.x = rs.x;
    pt.y = rs.y;
    pt.approach_yaw = rs.yaw;
    pt.tag_size = rs.tag_size;
    resp->rail_starts.push_back(pt);
  }
}

// ── State machine: coarse approach via Nav2 ──

void RailApproachNode::start_coarse_approach(const RailStart& rail) {
  state_ = State::COARSE_APPROACH;
  // Iter-11 / Option B: fresh approach attempt — clear any stale
  // reject reason so the state topic reflects this run only.
  last_reject_reason_ = "none";
  last_reject_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  // Iter-46 Paso 1.b: also clear PI state here so a repeated service
  // call doesn't inherit integral from a prior attempt. FINE_SERVOING
  // entry resets again as defence-in-depth.
  fine_servo_state_.reset();
  last_fine_servo_tick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  // Iter-12 / Option C: drop any prior-approach solvePnP samples so
  // the median filter doesn't bias the first fine_servoing ticks.
  pnp_filter_.reset();
  // Iter-13 / Option D: arm the fine-servo deadline only when the
  // state actually enters FINE_SERVOING (first detection). A zero
  // timestamp here means "not yet armed".
  fine_servo_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Iter-15: if the robot is already within acquisition range of the
  // tag, skip Nav2 coarse_approach entirely and go straight to
  // TAG_ACQUISITION. Two motivations:
  //   1. Floor tags carry yaw=0 in markers_registry (their face
  //      normal is +Z). The old code computed the Nav2 standoff pose
  //      as rail.x - standoff·cos(rail.yaw) = rail.x - standoff,
  //      which for wp04 (rail=(4, 0), robot=(5.2, 0, π)) pointed the
  //      robot to (3.5, 0) with goal yaw 0 — i.e. asking the robot to
  //      rotate 180° and drive PAST the tag. Nav2's forward-only MPPI
  //      (vx_min ≈ 0) refused the path (observed: "follow_path" halt,
  //      code=5).
  //   2. Every Round-44 waypoint teleports the robot to a pose where
  //      the tag is already ≤ 1.5 m away, which is well inside
  //      fine_servoing's operating range (range_min_m=0.05,
  //      range_max_m=5.0). Nav2's coarse phase adds time + failure
  //      surface for no precision gain.
  // Check distance to the registered tag using the brain's map→base_link
  // TF (ekf_global publishes both legs; available as soon as EKF has a
  // pose estimate, which the harness's _sync_brain_to_gt guarantees).
  // Iter-25 R3 (revised): the pre-alignment contract is satisfied at the
  // WAYPOINT level — every rail_approach waypoint is preceded by an
  // explicit nav2_prealign waypoint (see waypoints_tagged_v4.yaml).
  // Requiring coarse_approach INSIDE rail_approach_node also rotates the
  // robot to a standoff pose whose yaw comes from the tag registry
  // (rail.yaw, baked for the sim's +Z-normal floor tags = 0), which
  // clashes with the harness-chosen prealign yaw (π for REAR entries).
  // Forward-only MPPI cannot resolve a 180° spin cleanly, so a blind
  // coarse phase stalls. Restore the skip when the robot is close.
  try {
    const auto map_to_base = tf_buffer_->lookupTransform(
        "map", base_frame_,
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(0.5));
    const double rx = map_to_base.transform.translation.x;
    const double ry = map_to_base.transform.translation.y;
    const double dist = std::hypot(rail.x - rx, rail.y - ry);
    if (dist <= coarse_skip_radius_) {
      RCLCPP_INFO(get_logger(),
          "Skipping Nav2 coarse_approach: robot at (%.2f, %.2f) is "
          "%.2f m from tag %d (≤ %.2f m threshold). Jumping straight "
          "to TAG_ACQUISITION.",
          rx, ry, dist, rail.id, coarse_skip_radius_);
      state_ = State::TAG_ACQUISITION;
      acquisition_start_ = now();
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(),
        "map→%s TF not yet available; falling through to Nav2 coarse "
        "(%s)", base_frame_.c_str(), e.what());
    // Fall through to Nav2 coarse_approach.
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    finish(false, "Nav2 not available");
    return;
  }

  // Compute standoff pose: rail position minus standoff along approach heading
  auto goal_msg = NavAction::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position.x = rail.x - coarse_standoff_ * std::cos(rail.yaw);
  goal_msg.pose.pose.position.y = rail.y - coarse_standoff_ * std::sin(rail.yaw);
  double half_yaw = rail.yaw / 2.0;
  goal_msg.pose.pose.orientation.z = std::sin(half_yaw);
  goal_msg.pose.pose.orientation.w = std::cos(half_yaw);

  RCLCPP_INFO(get_logger(), "Nav2 goal: (%.2f, %.2f) yaw=%.2f",
              goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, rail.yaw);

  auto send_opts = rclcpp_action::Client<NavAction>::SendGoalOptions();
  send_opts.result_callback = std::bind(&RailApproachNode::on_nav_result, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_opts);
}

void RailApproachNode::on_nav_result(const NavGoalHandle::WrappedResult& result) {
  if (state_ != State::COARSE_APPROACH) return;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Coarse approach complete, acquiring tag %d", target_tag_id_);
    state_ = State::TAG_ACQUISITION;
    acquisition_start_ = now();
  } else {
    RCLCPP_ERROR(get_logger(), "Nav2 coarse approach failed (code=%d)", static_cast<int>(result.code));
    finish(false, "Nav2 coarse approach failed");
  }
}

// ── Detection callback: drives TAG_ACQUISITION and FINE_SERVOING ──

void RailApproachNode::on_detection(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {

  if (state_ != State::TAG_ACQUISITION && state_ != State::FINE_SERVOING) return;
  if (!camera_info_received_) return;

  // Find target tag in detections
  for (const auto& det : msg->detections) {
    if (det.id != target_tag_id_) continue;

    // Extract pixel corners
    std::vector<cv::Point2d> corners;
    for (const auto& c : det.corners) {
      corners.emplace_back(c.x, c.y);
    }
    if (corners.size() != 4) continue;

    // Get tag size for this rail start
    double tag_size = default_tag_size_;
    auto it = rail_starts_.find(target_tag_id_);
    if (it != rail_starts_.end()) {
      tag_size = it->second.tag_size;
    }

    if (state_ == State::TAG_ACQUISITION) {
      RCLCPP_INFO(get_logger(), "Tag %d acquired, starting fine servoing", target_tag_id_);
      state_ = State::FINE_SERVOING;
      tag_last_seen_ = now();
      // Iter-13 / Option D: stamp the fine-servo budget start.
      fine_servo_start_ = now();
      // Iter-46 Paso 1.b: fresh PI state for each approach. Stale integral
      // from a prior wp (or a reject-then-reacquire cycle) would push the
      // robot in the wrong direction at first contact.
      fine_servo_state_.reset();
      last_fine_servo_tick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    process_fine_servoing(det.id, corners, tag_size);
    return;
  }

  // Target tag not found in this frame
  if (state_ == State::TAG_ACQUISITION) {
    double elapsed = (now() - acquisition_start_).seconds();
    if (elapsed > acquisition_timeout_) {
      finish(false, "Tag " + std::to_string(target_tag_id_) + " not found within timeout");
    }
  } else if (state_ == State::FINE_SERVOING) {
    double since_last = (now() - tag_last_seen_).seconds();
    if (since_last > tag_loss_timeout_) {
      stop_robot();
      if (since_last > tag_reacquire_timeout_) {
        finish(false, "Tag lost during fine servoing");
      }
    }
  }
}

// ── Fine servoing: visual feedback loop ──

void RailApproachNode::process_fine_servoing(
    int /*tag_id*/, const std::vector<cv::Point2d>& corners, double tag_size) {

  tag_last_seen_ = now();

  // Iter-13 / Option D: enforce an upper bound on fine_servoing
  // duration. The legacy design relied on the harness-level deadline
  // (NAV_TIMEOUT_S × 1.5), which meant a servo oscillating inside the
  // 2 cm ring but never latching settle_frames_ stayed "driving"
  // forever from rail_approach's perspective. Aborting here lets the
  // arbiter and operator dashboard see the failure as its own class
  // rather than a generic nav timeout.
  if (max_fine_duration_ > 0.0 &&
      fine_servo_start_.nanoseconds() > 0) {
    const double elapsed = (now() - fine_servo_start_).seconds();
    if (elapsed > max_fine_duration_) {
      RCLCPP_WARN(get_logger(),
          "fine_servo_timeout after %.1f s — aborting approach", elapsed);
      last_reject_reason_ = "fine_servo_timeout";
      last_reject_stamp_ = now();
      finish(false, "fine_servo timeout: " +
             std::to_string(elapsed) + " s > " +
             std::to_string(max_fine_duration_) + " s");
      return;
    }
  }

  // Resolve the camera→base_link transform once per tick; the
  // controller itself is ROS-free and accepts a 4×4 homogeneous matrix.
  geometry_msgs::msg::TransformStamped cam_to_base_msg;
  try {
    cam_to_base_msg = tf_buffer_->lookupTransform(
        base_frame_, camera_frame_,
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(0.5));
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup %s->%s failed: %s",
        camera_frame_.c_str(), base_frame_.c_str(), e.what());
    last_reject_reason_ = "tf_missing";
    last_reject_stamp_ = now();
    return;
  }
  const tf2::Quaternion q(
      cam_to_base_msg.transform.rotation.x,
      cam_to_base_msg.transform.rotation.y,
      cam_to_base_msg.transform.rotation.z,
      cam_to_base_msg.transform.rotation.w);
  const tf2::Matrix3x3 R(q);
  cv::Matx44d cam_to_base = cv::Matx44d::eye();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) cam_to_base(r, c) = R[r][c];
  }
  cam_to_base(0, 3) = cam_to_base_msg.transform.translation.x;
  cam_to_base(1, 3) = cam_to_base_msg.transform.translation.y;
  cam_to_base(2, 3) = cam_to_base_msg.transform.translation.z;

  FineServoParams params;
  params.fx = fx_;
  params.fy = fy_;
  params.cx = cx_;
  params.cy = cy_;
  params.tag_size_m = tag_size;
  params.desired_offset_x = desired_offset_x_;
  params.desired_offset_y = desired_offset_y_;
  params.tolerance_xy = tolerance_xy_;
  params.tolerance_yaw = tolerance_yaw_;
  params.kp_linear = kp_linear_;
  params.kp_lateral = kp_lateral_;
  params.kp_yaw = kp_yaw_;
  params.ki_linear = ki_linear_;
  params.stiction_ff_vel_mps = stiction_ff_vel_mps_;
  params.max_linear_mps = max_fine_linear_;
  params.max_angular_rps = max_fine_angular_;
  params.check_yaw_convergence = check_yaw_convergence_;

  // Iter-12 / Option C: solvePnP → median filter → compute. Keeps the
  // control law's stateless contract while damping the per-frame
  // jitter solvePnP exhibits on edge-on floor tags.
  cv::Vec3d tvec, rvec;
  FineServoVerdict pnp_verdict = FineServoVerdict::OK;
  if (!solvepnp_tag(corners, params, tvec, rvec, pnp_verdict)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "fine_servo reject: %s", verdict_to_str(pnp_verdict));
    last_reject_reason_ = verdict_to_str(pnp_verdict);
    last_reject_stamp_ = now();
    return;
  }
  pnp_filter_.push(tvec, rvec);
  cv::Vec3d tvec_used = pnp_filter_.filled()
      ? pnp_filter_.tvec_median() : tvec;
  const cv::Vec3d rvec_used = pnp_filter_.filled()
      ? pnp_filter_.rvec_median() : rvec;

  // Iter-44 Fase 2 Arch A: override forward component (tvec.z in cam-
  // optical) with a TF+registry estimate when enabled. Lateral (tvec.x)
  // and rvec stay from PnP (well-conditioned even in grazing). The
  // TF lookup is map→camera_frame; the tag world pose comes from the
  // registry entry for the target tag. We only override when the lookup
  // succeeds within registry_max_stale_s (defensive: EKF may be stale
  // in the middle of a set_pose hand-off).
  if (use_registry_longitudinal_ && state_ == State::FINE_SERVOING) {
    auto rit = rail_starts_.find(target_tag_id_);
    if (rit != rail_starts_.end()) {
      const auto& rail = rit->second;
      try {
        const auto map_to_cam = tf_buffer_->lookupTransform(
            camera_frame_, "map",
            rclcpp::Time(0, 0, RCL_ROS_TIME),
            rclcpp::Duration::from_seconds(0.2));
        const double age = (now() - rclcpp::Time(map_to_cam.header.stamp)).seconds();
        if (age <= registry_max_stale_s_) {
          // Build tag in cam-optical frame: p_cam = R_cam_from_map * p_map + t.
          const tf2::Quaternion q(
              map_to_cam.transform.rotation.x,
              map_to_cam.transform.rotation.y,
              map_to_cam.transform.rotation.z,
              map_to_cam.transform.rotation.w);
          const tf2::Matrix3x3 R(q);
          const double tx = map_to_cam.transform.translation.x;
          const double ty = map_to_cam.transform.translation.y;
          const double tz = map_to_cam.transform.translation.z;
          // Floor tag z=0.002 m (from markers_registry.yaml). Wall tag
          // z=0.145 m. RailStart only carries x/y/yaw so we infer z from
          // the tag_size convention: rail_approach targets are floor tags.
          const double tag_z_world = 0.002;
          const double cam_x =
              R[0][0]*rail.x + R[0][1]*rail.y + R[0][2]*tag_z_world + tx;
          // const double cam_y =  // computed but unused; keep for clarity
          //     R[1][0]*rail.x + R[1][1]*rail.y + R[1][2]*tag_z_world + ty;
          (void)ty;
          const double cam_z =
              R[2][0]*rail.x + R[2][1]*rail.y + R[2][2]*tag_z_world + tz;
          // cam_z is the forward distance from camera to tag in optical
          // frame — exactly the quantity solvePnP puts in tvec[2]. We
          // swap it in, keeping tvec[0] (lateral) and rvec (yaw) from PnP
          // where they are well-conditioned.
          if (cam_z > 0.05 && cam_z < 5.0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "registry_longitudinal: pnp_z=%.3f → registry_z=%.3f (Δ=%+.3f m, cam_x=%.3f)",
                tvec_used[2], cam_z, cam_z - tvec_used[2], cam_x);
            tvec_used[2] = cam_z;
          }
        }
      } catch (const std::exception& e) {
        // Fall through; keep PnP tvec. Not an error — TF may not yet be
        // ready in the first ~1 s after state machine enters FINE_SERVOING.
      }
    }
  }

  // Iter-46 Paso 1.b: compute dt since last fine-servo tick for the
  // integral term. First tick of a new FINE_SERVOING episode has
  // last_fine_servo_tick_ as zero — fine_servo_compute skips
  // integration when dt_s <= 0, so that case is safe.
  const rclcpp::Time now_ts = now();
  double dt_s = 0.0;
  if (last_fine_servo_tick_.nanoseconds() > 0) {
    dt_s = (now_ts - last_fine_servo_tick_).seconds();
    // Guard against clock resets / huge gaps: cap dt at 1 s so a stall
    // doesn't dump an enormous integral contribution on the next tick.
    if (dt_s > 1.0 || dt_s < 0.0) dt_s = 0.0;
  }
  last_fine_servo_tick_ = now_ts;
  const auto out = fine_servo_compute(
      tvec_used, rvec_used, cam_to_base, params,
      fine_servo_state_, dt_s);
  if (out.verdict != FineServoVerdict::OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "fine_servo reject: %s (range=%.3f m)",
        verdict_to_str(out.verdict), out.range_m);
    last_reject_reason_ = verdict_to_str(out.verdict);
    last_reject_stamp_ = now();
    return;
  }
  // Happy path clears the last reject so operators see the recovery.
  last_reject_reason_ = "none";

  // Publish target pose for visualization.
  geometry_msgs::msg::PoseStamped target_msg;
  target_msg.header.stamp = now();
  target_msg.header.frame_id = base_frame_;
  target_msg.pose.position.x = out.tag_x_base;
  target_msg.pose.position.y = out.tag_y_base;
  target_msg.pose.position.z = out.tag_z_base;
  target_pose_pub_->publish(target_msg);

  // Settle-frame counter lives in the node (the controller is stateless).
  if (out.in_tolerance) {
    settle_count_++;
    if (settle_count_ >= settle_frames_) {
      stop_robot();
      RCLCPP_INFO(get_logger(),
          "Settled! error: x=%.4f y=%.4f yaw=%.4f",
          out.error_x, out.error_y, out.error_yaw);
      finish(true, "Precision approach complete",
             out.error_x, out.error_y, out.error_yaw);
      return;
    }
  } else {
    settle_count_ = 0;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = out.cmd_linear_mps;
  cmd.angular.z = out.cmd_angular_rps;
  cmd_pub_->publish(cmd);
}

// ── Finish (success or failure) ──

void RailApproachNode::finish(bool success, const std::string& message,
                               double err_x, double err_y, double err_yaw) {
  stop_robot();

  if (state_ == State::COARSE_APPROACH && nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
    nav_goal_handle_ = nullptr;
  }

  // Iter-17: distinguish success (SETTLED) from failure (ABORTED) so the
  // test harness + operator dashboard can detect a finished approach
  // without polling to NAV_TIMEOUT. Previous code collapsed both outcomes
  // to IDLE, which made a tag-not-found failure look identical to
  // "ready for the next approach" and forced the harness to wait the
  // full 270 s budget before reporting the waypoint. on_execute now
  // accepts IDLE, SETTLED, or ABORTED as ready-for-new-call states.
  state_ = success ? State::SETTLED : State::ABORTED;
  target_tag_id_ = -1;
  settle_count_ = 0;

  if (success) {
    RCLCPP_INFO(get_logger(), "Approach complete: %s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Approach failed: %s", message.c_str());
  }

  (void)err_x; (void)err_y; (void)err_yaw;
}

void RailApproachNode::stop_robot() {
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

// ── Status publishing ──

void RailApproachNode::publish_status() {
  // Dual label: `state` carries the mode_arbiter-compatible bucket and
  // `detail` carries the internal fine-grained state.
  //
  // Bucketing: only FINE_SERVOING counts as "driving" because that's the
  // phase in which rail_approach owns cmd_vel. During COARSE_APPROACH and
  // TAG_ACQUISITION the Nav2 stack is the active publisher (rail_approach
  // merely requested the Nav2 goal), so the arbiter should keep source=NAV
  // and relay cmd_vel_nav — otherwise cmd_vel stays zero and the coarse
  // approach never finishes.
  const std::string detail = state_name();
  std::string arbiter_state = detail;
  switch (state_) {
    case State::FINE_SERVOING:
      arbiter_state = "driving";
      break;
    default:
      break;  // idle/coarse_approach/tag_acquisition/settled/aborted pass through.
  }

  // Iter-11 / Option B: expose the last rejection that the fine servo
  // emitted so consumers (harness, operator dashboard, mode_arbiter)
  // can tell WHY an approach is stalled without digging through
  // WARN_THROTTLE log lines. "none" means the controller last reported
  // OK (or has never been stepped — the node resets this to "none" on
  // each start_coarse_approach).
  std_msgs::msg::String msg;
  msg.data = "{\"state\":\"" + arbiter_state +
             "\",\"detail\":\"" + detail +
             "\",\"target_tag\":" + std::to_string(target_tag_id_) +
             ",\"last_reject_reason\":\"" + last_reject_reason_ + "\"" +
             ",\"last_reject_age_s\":" + last_reject_age_str() + "}";
  status_pub_->publish(msg);
}

std::string RailApproachNode::last_reject_age_str() const {
  if (last_reject_reason_ == "none" || last_reject_stamp_.nanoseconds() == 0) {
    return "null";
  }
  const auto dt = (now() - last_reject_stamp_).seconds();
  // JSON doesn't allow NaN/Inf; clamp to a reasonable max.
  if (!std::isfinite(dt) || dt < 0.0 || dt > 1e6) return "null";
  // Round to 2 decimals without pulling in <iomanip>.
  const double rounded = std::round(dt * 100.0) / 100.0;
  return std::to_string(rounded);
}

std::string RailApproachNode::state_name() const {
  switch (state_) {
    case State::IDLE: return "idle";
    case State::COARSE_APPROACH: return "coarse_approach";
    case State::TAG_ACQUISITION: return "tag_acquisition";
    case State::FINE_SERVOING: return "fine_servoing";
    case State::SETTLED: return "settled";
    case State::ABORTED: return "aborted";
  }
  return "unknown";
}

}  // namespace agv_rail_approach
