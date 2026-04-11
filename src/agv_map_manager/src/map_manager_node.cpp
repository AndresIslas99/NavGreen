// agv_map_manager — C++17 ROS2 node for map persistence and zone management
//
// Services:
//   map_manager/save_map   — save current occupancy grid to disk
//   map_manager/load_map   — load map from disk via nav2 map_server
//   map_manager/update_zone — persist keepout/speed zones alongside map

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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

    map_dir_ = this->get_parameter("map_dir").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    cuvslam_enabled_ = this->get_parameter("cuvslam_enabled").as_bool();

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

    // Event publisher: emitted after any successful load_map so the auto_init
    // orchestrator can begin its relocalization sequence.
    map_loaded_pub_ = this->create_publisher<std_msgs::msg::String>(
      "maps/loaded", rclcpp::QoS(1).transient_local());

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

    RCLCPP_INFO(get_logger(), "Map manager ready, map_dir=%s", map_dir_.c_str());
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
    auto yaml_path = map_dir_ + "/" + name + ".yaml";

    if (!fs::exists(yaml_path)) {
      res->success = false;
      res->message = "Map not found: " + yaml_path;
      return;
    }

    if (load_map_internal(yaml_path)) {
      res->success = true;
      res->message = "Map loaded: " + name;
      // Fire the maps/loaded event so auto_init_orchestrator starts its
      // relocalization sequence (load cuVSLAM DB + wait for tag + localize_in_map).
      publish_map_loaded_event(name);
    } else {
      res->success = false;
      res->message = "Failed to load map via nav2";
    }
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

  // Emit a maps/loaded event so downstream nodes (auto_init_orchestrator) can
  // begin their localization sequence. The payload is the map name (no extension).
  void publish_map_loaded_event(const std::string& map_name_or_path) {
    std_msgs::msg::String msg;
    // Normalize to just the stem if a path was passed
    msg.data = fs::path(map_name_or_path).stem().string();
    map_loaded_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published maps/loaded event: '%s'", msg.data.c_str());
  }

  std::string map_dir_;
  std::string map_topic_;
  bool cuvslam_enabled_{false};
  rclcpp::Service<agv_interfaces::srv::SaveMap>::SharedPtr save_srv_;
  rclcpp::Service<agv_interfaces::srv::LoadMap>::SharedPtr load_srv_;
  rclcpp::Service<agv_interfaces::srv::UpdateZone>::SharedPtr zone_srv_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr nav2_load_client_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr cuvslam_save_client_;
  rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::FilePath>::SharedPtr cuvslam_load_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_loaded_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapManagerNode>());
  rclcpp::shutdown();
  return 0;
}
