// agv_map_manager — C++17 ROS2 node for map persistence and zone management
//
// Services:
//   map_manager/save_map   — save current occupancy grid to disk
//   map_manager/load_map   — load map from disk via nav2 map_server
//   map_manager/update_zone — persist keepout/speed zones alongside map

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <agv_interfaces/srv/save_map.hpp>
#include <agv_interfaces/srv/load_map.hpp>
#include <agv_interfaces/srv/update_zone.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/file_path.hpp>

namespace fs = std::filesystem;

class MapManagerNode : public rclcpp::Node {
public:
  MapManagerNode() : Node("map_manager") {
    this->declare_parameter("map_dir", "");
    this->declare_parameter("default_map", "");
    this->declare_parameter("map_topic", "/agv/map");
    // When true, save/load also calls /visual_slam/save_map and /visual_slam/load_map
    // so cuVSLAM can relocalize against a prior keyframe database at boot time.
    // Disable only for pure HIL/sim where cuVSLAM is not running.
    this->declare_parameter("cuvslam_enabled", true);
    // When true, on a successful save_map we also trigger
    // /agv/zed/save_area_memory so the SDK Area Memory landmark DB is
    // refreshed alongside the cuVSLAM keyframe DB. Disable only for HIL/sim
    // where the ZED wrapper is not running with pos_tracking enabled.
    this->declare_parameter("zed_area_save_enabled", true);
    // The single landing-pad path that the ZED wrapper knows about
    // (pos_tracking.area_memory_db_path). On save we copy this file to
    // `<map_dir>/<name>.area`; on load we copy `<name>.area` back here and
    // trigger reset_pos_tracking so the SDK re-reads the swapped contents.
    this->declare_parameter("zed_area_landing_path",
      std::string("/home/orza/agv_data/maps/.current.area"));
    // Auto-save period for the area memory while a real map is loaded.
    // Defends against crashes/reboots losing the in-RAM landmark DB the
    // operator has accumulated between explicit Save Map clicks.
    // 0 disables. Default 180 s (3 min).
    this->declare_parameter("area_memory_autosave_period_s", 180);

    map_dir_ = this->get_parameter("map_dir").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    cuvslam_enabled_ = this->get_parameter("cuvslam_enabled").as_bool();
    zed_area_save_enabled_ = this->get_parameter("zed_area_save_enabled").as_bool();
    zed_area_landing_path_ = this->get_parameter("zed_area_landing_path").as_string();
    area_memory_autosave_period_s_ =
      this->get_parameter("area_memory_autosave_period_s").as_int();

    if (map_dir_.empty()) {
      RCLCPP_FATAL(get_logger(), "map_dir parameter is required");
      throw std::runtime_error("map_dir not set");
    }
    fs::create_directories(map_dir_);

    // Services
    save_srv_ = this->create_service<agv_interfaces::srv::SaveMap>(
      "map_manager/save_map",
      std::bind(&MapManagerNode::on_save_map, this,
                std::placeholders::_1, std::placeholders::_2));

    load_srv_ = this->create_service<agv_interfaces::srv::LoadMap>(
      "map_manager/load_map",
      std::bind(&MapManagerNode::on_load_map, this,
                std::placeholders::_1, std::placeholders::_2));

    zone_srv_ = this->create_service<agv_interfaces::srv::UpdateZone>(
      "map_manager/update_zone",
      std::bind(&MapManagerNode::on_update_zone, this,
                std::placeholders::_1, std::placeholders::_2));

    // Nav2 LoadMap client
    nav2_load_client_ = this->create_client<nav2_msgs::srv::LoadMap>(
      "map_server/load_map");

    // cuVSLAM save/load map clients (live in the /visual_slam namespace, not ours)
    if (cuvslam_enabled_) {
      cuvslam_save_client_ = this->create_client<isaac_ros_visual_slam_interfaces::srv::FilePath>(
        "/visual_slam/save_map");
      cuvslam_load_client_ = this->create_client<isaac_ros_visual_slam_interfaces::srv::FilePath>(
        "/visual_slam/load_map");
    }

    // ZED SDK Area Memory service clients. The wrapper is launched under
    // namespace /agv with node name 'zed', so services live at /agv/zed/*.
    // On Save: /agv/zed/save_area_memory flushes the in-RAM landmark DB to
    // the path configured in pos_tracking.area_memory_db_path (the landing
    // pad). We then copy that file to <map_dir>/<name>.area so each map has
    // its own landmark DB. On Load: we copy <name>.area back onto the
    // landing pad and call /agv/zed/reset_pos_tracking — the (patched)
    // wrapper re-reads the param and re-invokes enablePositionalTracking,
    // which loads the swapped file from disk.
    if (zed_area_save_enabled_) {
      zed_save_area_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/agv/zed/save_area_memory");
      zed_reset_pt_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/agv/zed/reset_pos_tracking");
    }

    // Service client for auto_init_orchestrator's synchronous save of
    // <map>_meta.json. Called from on_save_map so the last-known-pose file
    // lands on disk at the moment of Save Map, not 30 s later when the
    // orchestrator's periodic save fires. Closes audit bug #2 (save/load
    // symmetry for per-map meta.json).
    orchestrator_save_pose_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/agv/localization/save_last_known_pose");

    // Event publisher: emitted after any successful load_map so the auto_init
    // orchestrator can begin its relocalization sequence.
    map_loaded_pub_ = this->create_publisher<std_msgs::msg::String>(
      "maps/loaded", rclcpp::QoS(1).transient_local());

    // Latched current_map topic — the dashboard header subscribes to this so
    // the operator can always see which map is active. transient_local means
    // any late subscriber gets the last value without waiting for an event.
    current_map_pub_ = this->create_publisher<std_msgs::msg::String>(
      "current_map", rclcpp::QoS(1).transient_local().reliable());

    // Listen to maps/loaded regardless of who publishes it (self or the
    // backend's boot-time publish) so current_map and ~/.agv/last_map stay
    // synchronized with whatever Nav2 actually loaded.
    map_loaded_sub_ = this->create_subscription<std_msgs::msg::String>(
      "maps/loaded", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_maps_loaded_event(msg->data);
      });

    // Load default map on startup
    auto default_map = this->get_parameter("default_map").as_string();
    if (!default_map.empty()) {
      RCLCPP_INFO(get_logger(), "Loading default map: %s", default_map.c_str());
      // Defer to allow nav2 to start
      auto timer = this->create_wall_timer(
        std::chrono::seconds(2),
        [this, default_map]() {
          load_map_internal(default_map);
          // One-shot timer
        });
    }

    // Auto-save timer — periodically flushes the Area Memory landmark DB to
    // disk while a real map is loaded, so a crash/reboot loses at most N
    // seconds of accumulated landmarks instead of the whole session.
    if (area_memory_autosave_period_s_ > 0) {
      autosave_timer_ = this->create_wall_timer(
        std::chrono::seconds(area_memory_autosave_period_s_),
        [this]() { auto_save_area_memory_tick(); });
    }

    RCLCPP_INFO(get_logger(),
      "Map manager ready, map_dir=%s, area_landing=%s, autosave=%ds",
      map_dir_.c_str(), zed_area_landing_path_.c_str(),
      area_memory_autosave_period_s_);
  }

private:
  void on_save_map(
    const agv_interfaces::srv::SaveMap::Request::SharedPtr req,
    agv_interfaces::srv::SaveMap::Response::SharedPtr res)
  {
    auto name = req->name;
    if (name.empty() || name.find('/') != std::string::npos || name.find("..") != std::string::npos) {
      res->success = false;
      res->message = "Invalid map name";
      return;
    }

    auto out_path = map_dir_ + "/" + name;
    RCLCPP_INFO(get_logger(), "Saving map to %s", out_path.c_str());

    // Use popen instead of system() — captures output, avoids shell injection
    std::string cmd = "ros2 run nav2_map_server map_saver_cli"
                      " -f '" + out_path + "'"
                      " -t '" + map_topic_ + "'"
                      " --ros-args -p save_map_timeout:=10.0 2>&1";

    std::string output;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      res->success = false;
      res->message = "Failed to launch map_saver_cli";
      return;
    }
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe)) { output += buf; }
    int ret = pclose(pipe);

    if (ret == 0) {
      res->success = true;
      res->message = "Map saved: " + name;
      RCLCPP_INFO(get_logger(), "Map saved: %s", name.c_str());
      // Also persist the cuVSLAM keyframe database so future sessions can
      // relocalize automatically. Non-blocking — the 2D map is already saved.
      save_cuvslam_map(name);
      // Also flush the ZED SDK Area Memory so cold-start (no AprilTag in
      // view) can relocalise via Path A0 in the orchestrator. Non-blocking.
      save_zed_area_memory(name);
      // Ask the localization orchestrator to write <name>_meta.json NOW so
      // Path C (last-known-pose fallback) is usable immediately after this
      // Save Map — without this, the next Load Map that falls through to
      // Path C would read a stale meta from a prior session. Best-effort,
      // non-blocking (the guards in the orchestrator may legitimately refuse).
      trigger_orchestrator_pose_save(name);
    } else {
      res->success = false;
      res->message = "map_saver_cli failed (exit " + std::to_string(ret) + "): " + output;
      RCLCPP_ERROR(get_logger(), "Map save failed: %s — %s", name.c_str(), output.c_str());
    }
  }

  void on_load_map(
    const agv_interfaces::srv::LoadMap::Request::SharedPtr req,
    agv_interfaces::srv::LoadMap::Response::SharedPtr res)
  {
    auto name = req->name;
    if (name.empty() || name.find('/') != std::string::npos ||
        name.find("..") != std::string::npos) {
      res->success = false;
      res->message = "Invalid map name";
      return;
    }
    auto yaml_path = map_dir_ + "/" + name + ".yaml";

    if (!fs::exists(yaml_path)) {
      res->success = false;
      res->message = "Map not found: " + yaml_path;
      return;
    }

    // Swap the per-map area memory file onto the wrapper's landing pad
    // BEFORE triggering nav2 load and the orchestrator cascade. The reset
    // happens after nav2 load succeeds so the SDK re-loads the landmark DB
    // at the same time the orchestrator starts its Path A0 check.
    swap_area_memory_for_map(name);

    if (load_map_internal(yaml_path)) {
      res->success = true;
      res->message = "Map loaded: " + name;
      // Ask the ZED wrapper to re-enable pos tracking so the (patched)
      // startPosTracking re-reads the param and the SDK re-loads the
      // .area file we just swapped onto the landing pad.
      reset_zed_pos_tracking();
      // Fire the maps/loaded event so auto_init_orchestrator starts its
      // relocalization sequence (load cuVSLAM DB + wait for tag + localize_in_map).
      publish_map_loaded_event(name);
    } else {
      res->success = false;
      res->message = "Failed to load map via nav2";
    }
  }

  // Copy `<map_dir>/<name>.area` onto the landing pad the ZED wrapper knows
  // about. If the per-map file does not exist yet (map was saved before
  // Area Memory integration, or this is the first save), we zero out the
  // landing pad so the SDK falls back to VO-only — without this, a load of
  // a map-without-area would inherit stale landmarks from whatever map was
  // active before.
  void swap_area_memory_for_map(const std::string& name) {
    if (zed_area_landing_path_.empty()) return;
    if (name == "default_empty") {
      // Placeholder map — clear the landing pad so we don't reuse the last
      // real map's landmarks as if they applied to the empty grid.
      try {
        std::ofstream(zed_area_landing_path_, std::ios::trunc);
      } catch (...) {}
      return;
    }
    fs::path per_map = fs::path(map_dir_) / (name + ".area");
    try {
      if (fs::exists(per_map) && fs::file_size(per_map) > 0) {
        fs::copy_file(per_map, zed_area_landing_path_,
                      fs::copy_options::overwrite_existing);
        RCLCPP_INFO(get_logger(),
          "Area Memory: swapped '%s' onto landing pad '%s' (%ld bytes)",
          per_map.c_str(), zed_area_landing_path_.c_str(),
          static_cast<long>(fs::file_size(zed_area_landing_path_)));
      } else {
        // No per-map file yet. Truncate the landing pad so the SDK starts
        // fresh for this map instead of reusing the previous map's DB.
        std::ofstream(zed_area_landing_path_, std::ios::trunc);
        RCLCPP_INFO(get_logger(),
          "Area Memory: no per-map file for '%s' — cleared landing pad",
          name.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
        "Area Memory swap failed for '%s': %s", name.c_str(), e.what());
    }
  }

  // Ask the ZED wrapper to reset positional tracking so the (patched)
  // startPosTracking re-reads the area memory file from disk. Fire and
  // forget — failures here are non-fatal; the orchestrator's Path A0 will
  // notice missing landmarks via cuVSLAM state anyway.
  void reset_zed_pos_tracking() {
    if (!zed_reset_pt_client_) return;
    if (!zed_reset_pt_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/agv/zed/reset_pos_tracking not ready — area memory will not be reloaded this cycle");
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    zed_reset_pt_client_->async_send_request(req,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        try {
          auto r = f.get();
          if (r->success) {
            RCLCPP_INFO(get_logger(),
              "ZED pos tracking reset (Area Memory reloaded from landing pad)");
          } else {
            RCLCPP_WARN(get_logger(),
              "ZED reset_pos_tracking reported failure: %s", r->message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "reset_pos_tracking exception: %s", e.what());
        }
      });
  }

  // Periodic flush of the in-RAM landmark DB to disk while a real map is
  // loaded. Called by autosave_timer_ every area_memory_autosave_period_s_.
  void auto_save_area_memory_tick() {
    if (last_seen_map_name_.empty() ||
        last_seen_map_name_ == "default_empty") {
      return;  // no real map → nothing worth persisting
    }
    RCLCPP_DEBUG(get_logger(),
      "Area Memory autosave tick for '%s'", last_seen_map_name_.c_str());
    save_zed_area_memory(last_seen_map_name_);
  }

  bool load_map_internal(const std::string& yaml_path) {
    if (!nav2_load_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "map_server/load_map service not available");
      return false;
    }

    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = yaml_path;

    auto future = nav2_load_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                            std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = future.get();
      if (result->result == 0) {
        RCLCPP_INFO(get_logger(), "Map loaded: %s", yaml_path.c_str());
        publish_map_loaded_event(yaml_path);
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "Map load failed: %s", yaml_path.c_str());
    return false;
  }

  void on_update_zone(
    const agv_interfaces::srv::UpdateZone::Request::SharedPtr req,
    agv_interfaces::srv::UpdateZone::Response::SharedPtr res)
  {
    // Zone persistence: store as simple JSON alongside maps
    auto zones_path = map_dir_ + "/zones.json";

    // Read existing zones
    std::vector<std::string> lines;
    if (fs::exists(zones_path)) {
      std::ifstream in(zones_path);
      std::string line;
      while (std::getline(in, line)) {
        // Simple: each line is a zone JSON
        if (line.find("\"zone_id\":\"" + req->zone_id + "\"") == std::string::npos) {
          lines.push_back(line);
        }
      }
    }

    if (!req->remove) {
      // Validate polygon geometry
      if (req->polygon_x.size() < 3) {
        res->success = false;
        res->message = "Polygon must have at least 3 vertices";
        return;
      }
      if (req->polygon_x.size() != req->polygon_y.size()) {
        res->success = false;
        res->message = "polygon_x and polygon_y must have the same length";
        return;
      }
      if (req->zone_type != "keepout" && req->zone_type != "speed") {
        res->success = false;
        res->message = "zone_type must be 'keepout' or 'speed'";
        return;
      }
      if (req->zone_type == "speed" && req->max_speed <= 0.0) {
        res->success = false;
        res->message = "Speed zone requires max_speed > 0";
        return;
      }

      // Add/update zone
      std::string zone_json = "{\"zone_id\":\"" + req->zone_id
        + "\",\"zone_type\":\"" + req->zone_type
        + "\",\"max_speed\":" + std::to_string(req->max_speed)
        + ",\"polygon_x\":[";
      for (size_t i = 0; i < req->polygon_x.size(); ++i) {
        if (i > 0) zone_json += ",";
        zone_json += std::to_string(req->polygon_x[i]);
      }
      zone_json += "],\"polygon_y\":[";
      for (size_t i = 0; i < req->polygon_y.size(); ++i) {
        if (i > 0) zone_json += ",";
        zone_json += std::to_string(req->polygon_y[i]);
      }
      zone_json += "]}";
      lines.push_back(zone_json);
    }

    // Write back
    std::ofstream out(zones_path);
    for (const auto& l : lines) {
      out << l << "\n";
    }

    res->success = true;
    res->message = req->remove ? "Zone removed" : "Zone updated";
    RCLCPP_INFO(get_logger(), "%s zone %s", req->remove ? "Removed" : "Updated",
                req->zone_id.c_str());
  }

  // Fire-and-forget helper to ask cuVSLAM to save its keyframe database to a
  // per-map subdirectory. Logs failure but does NOT block the main save path —
  // the 2D occupancy grid is saved either way.
  void save_cuvslam_map(const std::string& name) {
    if (!cuvslam_enabled_ || !cuvslam_save_client_) return;
    if (!cuvslam_save_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "cuVSLAM /visual_slam/save_map not available — skipping keyframe DB save for '%s'",
        name.c_str());
      return;
    }
    const auto cuvslam_dir = map_dir_ + "/" + name + "_cuvslam";
    try {
      fs::create_directories(cuvslam_dir);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create cuVSLAM map dir %s: %s",
                   cuvslam_dir.c_str(), e.what());
      return;
    }
    auto req = std::make_shared<isaac_ros_visual_slam_interfaces::srv::FilePath::Request>();
    req->file_path = cuvslam_dir;
    // Async with callback — do not block the save service
    cuvslam_save_client_->async_send_request(req,
      [this, name, cuvslam_dir](rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedFuture f) {
        try {
          auto r = f.get();
          if (r->success) {
            RCLCPP_INFO(get_logger(), "cuVSLAM keyframe DB saved: %s", cuvslam_dir.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "cuVSLAM save_map reported failure for '%s'", name.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "cuVSLAM save_map exception: %s", e.what());
        }
      });
  }

  // Ask the ZED wrapper to flush its in-RAM Area Memory landmark DB to the
  // landing pad, then (asynchronously, on the service response) copy the
  // landing pad to a per-map file `<map_dir>/<name>.area`. This is how a
  // map's landmark DB becomes persistent and distinguishable from other
  // maps. Called on explicit Save Map and on the periodic autosave tick.
  void save_zed_area_memory(const std::string& name) {
    if (!zed_area_save_enabled_ || !zed_save_area_client_) return;
    if (name.empty() || name == "default_empty") return;
    if (!zed_save_area_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/agv/zed/save_area_memory not available — skipping ZED Area Memory save for '%s'",
        name.c_str());
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    zed_save_area_client_->async_send_request(req,
      [this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        try {
          auto r = f.get();
          if (r->success) {
            RCLCPP_INFO(get_logger(),
              "ZED Area Memory saved to landing pad (map='%s'): %s",
              name.c_str(), r->message.c_str());
            copy_landing_pad_to_per_map(name);
          } else {
            RCLCPP_WARN(get_logger(),
              "ZED Area Memory save reported failure (map='%s'): %s",
              name.c_str(), r->message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(),
            "ZED Area Memory save exception: %s", e.what());
        }
      });
  }

  // Call /agv/localization/save_last_known_pose to flush <map>_meta.json at
  // the moment of Save Map. The orchestrator applies its own safety guards
  // (never-called, implausible-distance) so failures here are not fatal — we
  // log them and let the periodic save retry later.
  void trigger_orchestrator_pose_save(const std::string& name) {
    if (!orchestrator_save_pose_client_) return;
    if (!orchestrator_save_pose_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/agv/localization/save_last_known_pose not ready — %s_meta.json will "
        "be written by the next periodic save instead", name.c_str());
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    orchestrator_save_pose_client_->async_send_request(req,
      [this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        try {
          auto r = f.get();
          if (r->success) {
            RCLCPP_INFO(get_logger(),
              "orchestrator wrote %s_meta.json: %s", name.c_str(), r->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(),
              "orchestrator refused pose save for '%s': %s",
              name.c_str(), r->message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(),
            "orchestrator pose save exception for '%s': %s", name.c_str(), e.what());
        }
      });
  }

  void copy_landing_pad_to_per_map(const std::string& name) {
    if (zed_area_landing_path_.empty()) return;
    try {
      if (!fs::exists(zed_area_landing_path_)) {
        RCLCPP_WARN(get_logger(),
          "Area Memory landing pad '%s' missing after save — nothing to copy",
          zed_area_landing_path_.c_str());
        return;
      }
      auto sz = fs::file_size(zed_area_landing_path_);
      if (sz == 0) {
        RCLCPP_WARN(get_logger(),
          "Area Memory landing pad is empty — SDK likely rejected the save");
        return;
      }
      fs::path per_map = fs::path(map_dir_) / (name + ".area");
      fs::path tmp = fs::path(map_dir_) / (name + ".area.tmp");
      fs::copy_file(zed_area_landing_path_, tmp,
                    fs::copy_options::overwrite_existing);
      fs::rename(tmp, per_map);
      RCLCPP_INFO(get_logger(),
        "Area Memory: persisted %ld bytes to '%s'",
        static_cast<long>(sz), per_map.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
        "Area Memory copy to per-map '%s.area' failed: %s",
        name.c_str(), e.what());
    }
  }

  // Emit a maps/loaded event so downstream nodes (auto_init_orchestrator) can
  // begin their localization sequence. The payload is the map name (no extension).
  void publish_map_loaded_event(const std::string& map_name_or_path) {
    std_msgs::msg::String msg;
    // Normalize to just the stem if a path was passed
    msg.data = fs::path(map_name_or_path).stem().string();
    map_loaded_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published maps/loaded event: '%s'", msg.data.c_str());
  }

  // Reacts to any maps/loaded event (own or from agv_ui_backend's boot-time
  // publish). Updates the latched current_map topic and persists the name to
  // ~/.agv/last_map so agv_start.sh can reload it on the next boot.
  //
  // "default_empty" is the placeholder map that agv_start.sh loads when the
  // operator has not saved anything yet. It must not be treated as a real
  // map: the dashboard header should still show "mapping…" (publish empty
  // string), and it must not be persisted as the last map.
  void on_maps_loaded_event(const std::string& name) {
    if (name.empty()) return;
    if (name == last_seen_map_name_) return;
    last_seen_map_name_ = name;

    const bool is_default = (name == "default_empty");
    std_msgs::msg::String out;
    out.data = is_default ? "" : name;
    current_map_pub_->publish(out);

    if (!is_default) {
      persist_last_map(name);
    }
  }

  // Writes the name to ${HOME}/.agv/last_map atomically (tmp + rename). A
  // missing HOME or an unwritable directory is logged but not fatal — the
  // live state is still correct.
  void persist_last_map(const std::string& name) {
    const char* home = std::getenv("HOME");
    if (!home || *home == '\0') {
      RCLCPP_WARN(get_logger(),
        "HOME is not set; cannot persist last_map='%s'", name.c_str());
      return;
    }
    fs::path dir = fs::path(home) / ".agv";
    fs::path target = dir / "last_map";
    fs::path tmp = dir / "last_map.tmp";
    try {
      fs::create_directories(dir);
      {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) {
          RCLCPP_WARN(get_logger(),
            "Failed to open %s for writing", tmp.c_str());
          return;
        }
        out << name << "\n";
      }
      fs::rename(tmp, target);
      RCLCPP_INFO(get_logger(),
        "Persisted last_map='%s' to %s", name.c_str(), target.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
        "Failed to persist last_map: %s", e.what());
    }
  }

  std::string map_dir_;
  std::string map_topic_;
  bool cuvslam_enabled_{false};
  bool zed_area_save_enabled_{false};
  std::string zed_area_landing_path_;
  int area_memory_autosave_period_s_{0};
  rclcpp::TimerBase::SharedPtr autosave_timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zed_reset_pt_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr orchestrator_save_pose_client_;
  rclcpp::Service<agv_interfaces::srv::SaveMap>::SharedPtr save_srv_;
  rclcpp::Service<agv_interfaces::srv::LoadMap>::SharedPtr load_srv_;
  rclcpp::Service<agv_interfaces::srv::UpdateZone>::SharedPtr zone_srv_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr nav2_load_client_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr cuvslam_save_client_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr cuvslam_load_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zed_save_area_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_loaded_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_map_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_loaded_sub_;
  std::string last_seen_map_name_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerNode>());
  rclcpp::shutdown();
  return 0;
}
