// agv_waypoint_manager — C++17 ROS2 node for mission storage and dispatch
//
// Services:
//   waypoint_manager/save     — persist mission to JSON file
//   waypoint_manager/list     — return all stored missions
//   waypoint_manager/execute  — execute mission via sequential navigate_to_pose goals

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <agv_interfaces/srv/save_waypoint.hpp>
#include <agv_interfaces/srv/list_missions.hpp>
#include <agv_interfaces/srv/execute_mission.hpp>
#include <agv_interfaces/msg/waypoint.hpp>
#include <agv_interfaces/msg/mission.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace fs = std::filesystem;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// Simple JSON helpers (no external JSON lib dependency)
namespace json_util {

std::string escape(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else out += c;
  }
  return out;
}

// Extract a string value from a simple JSON object
std::string get_string(const std::string& json, const std::string& key) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return "";
  pos = json.find(':', pos);
  if (pos == std::string::npos) return "";
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) return "";
  auto end = json.find('"', pos + 1);
  if (end == std::string::npos) return "";
  return json.substr(pos + 1, end - pos - 1);
}

double get_double(const std::string& json, const std::string& key) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return 0.0;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return 0.0;
  return std::stod(json.substr(pos + 1));
}

}  // namespace json_util

class WaypointManagerNode : public rclcpp::Node {
public:
  WaypointManagerNode() : Node("waypoint_manager") {
    this->declare_parameter("missions_file", "");
    this->declare_parameter("default_speed", 0.3);

    missions_file_ = this->get_parameter("missions_file").as_string();
    default_speed_ = this->get_parameter("default_speed").as_double();

    if (missions_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "missions_file parameter is required");
      throw std::runtime_error("missions_file not set");
    }

    // Create parent directory if needed
    auto parent = fs::path(missions_file_).parent_path();
    if (!parent.empty()) fs::create_directories(parent);
    if (!fs::exists(missions_file_)) {
      std::ofstream(missions_file_) << "[]";
    }

    // Services
    save_srv_ = this->create_service<agv_interfaces::srv::SaveWaypoint>(
      "waypoint_manager/save",
      std::bind(&WaypointManagerNode::on_save, this,
                std::placeholders::_1, std::placeholders::_2));

    list_srv_ = this->create_service<agv_interfaces::srv::ListMissions>(
      "waypoint_manager/list",
      std::bind(&WaypointManagerNode::on_list, this,
                std::placeholders::_1, std::placeholders::_2));

    execute_srv_ = this->create_service<agv_interfaces::srv::ExecuteMission>(
      "waypoint_manager/execute",
      std::bind(&WaypointManagerNode::on_execute, this,
                std::placeholders::_1, std::placeholders::_2));

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Mission status publisher (JSON: mission_id, current_waypoint, total, state)
    status_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_manager/status", 10);

    // Cancel subscription
    cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "waypoint_manager/cancel", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && mission_running_.load()) {
          mission_cancel_.store(true);
          RCLCPP_WARN(get_logger(), "Mission cancel requested");
        }
      });

    RCLCPP_INFO(get_logger(), "Waypoint manager ready, file=%s", missions_file_.c_str());
  }

  ~WaypointManagerNode() override {
    mission_cancel_.store(true);
    if (exec_thread_.joinable()) exec_thread_.join();
  }

private:
  // ── Save mission ──
  void on_save(
    const agv_interfaces::srv::SaveWaypoint::Request::SharedPtr req,
    agv_interfaces::srv::SaveWaypoint::Response::SharedPtr res)
  {
    auto id = req->mission_id;
    if (id.empty()) {
      // Generate simple ID
      id = "m" + std::to_string(
        std::chrono::system_clock::now().time_since_epoch().count() % 100000000);
    }

    // Build JSON line for this mission
    std::ostringstream oss;
    oss << "{\"id\":\"" << json_util::escape(id)
        << "\",\"name\":\"" << json_util::escape(req->mission_name)
        << "\",\"created\":" << std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch()).count()
        << ",\"waypoints\":[";

    for (size_t i = 0; i < req->waypoints.size(); ++i) {
      if (i > 0) oss << ",";
      const auto& wp = req->waypoints[i];
      oss << "{\"x\":" << wp.x << ",\"y\":" << wp.y << ",\"theta\":" << wp.theta
          << ",\"action\":\"" << json_util::escape(wp.action) << "\""
          << ",\"pause_sec\":" << wp.pause_sec << "}";
    }
    oss << "]}";

    // Append to file (simple line-per-mission format)
    {
      std::ofstream out(missions_file_, std::ios::app);
      out << oss.str() << "\n";
    }

    res->success = true;
    res->mission_id = id;
    RCLCPP_INFO(get_logger(), "Mission saved: %s (%zu waypoints)",
                req->mission_name.c_str(), req->waypoints.size());
  }

  // ── List missions ──
  void on_list(
    const agv_interfaces::srv::ListMissions::Request::SharedPtr /*req*/,
    agv_interfaces::srv::ListMissions::Response::SharedPtr res)
  {
    std::ifstream in(missions_file_);
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] != '{') continue;

      agv_interfaces::msg::Mission m;
      m.id = json_util::get_string(line, "id");
      m.name = json_util::get_string(line, "name");
      m.created = json_util::get_double(line, "created");
      // Note: waypoint parsing from JSON is simplified — production would use a proper JSON lib
      res->missions.push_back(m);
    }
    RCLCPP_INFO(get_logger(), "Listed %zu missions", res->missions.size());
  }

  // ── Execute mission (non-blocking) ──
  void on_execute(
    const agv_interfaces::srv::ExecuteMission::Request::SharedPtr req,
    agv_interfaces::srv::ExecuteMission::Response::SharedPtr res)
  {
    if (mission_running_.load()) {
      res->success = false;
      res->message = "A mission is already running";
      return;
    }

    // Find mission by ID
    std::ifstream in(missions_file_);
    std::string line;
    std::string found_line;
    while (std::getline(in, line)) {
      if (line.find("\"id\":\"" + req->mission_id + "\"") != std::string::npos) {
        found_line = line;
        break;
      }
    }

    if (found_line.empty()) {
      res->success = false;
      res->message = "Mission not found: " + req->mission_id;
      return;
    }

    // Parse waypoints
    std::vector<std::tuple<double, double, double>> waypoints;
    auto wp_pos = found_line.find("\"waypoints\":[");
    if (wp_pos != std::string::npos) {
      auto wp_str = found_line.substr(wp_pos);
      size_t search_pos = 0;
      while (true) {
        auto x_pos = wp_str.find("\"x\":", search_pos);
        if (x_pos == std::string::npos) break;
        double x = std::stod(wp_str.substr(x_pos + 4));
        auto y_pos = wp_str.find("\"y\":", x_pos);
        double y = std::stod(wp_str.substr(y_pos + 4));
        auto t_pos = wp_str.find("\"theta\":", y_pos);
        double theta = std::stod(wp_str.substr(t_pos + 8));
        waypoints.emplace_back(x, y, theta);
        search_pos = t_pos + 10;
      }
    }

    if (waypoints.empty()) {
      res->success = false;
      res->message = "Mission has no waypoints";
      return;
    }

    // Respond immediately — execution runs in background
    res->success = true;
    res->message = "Mission started (" + std::to_string(waypoints.size()) + " waypoints)";

    // Launch execution thread
    if (exec_thread_.joinable()) exec_thread_.join();
    mission_cancel_.store(false);
    mission_running_.store(true);
    std::string mission_id = req->mission_id;

    exec_thread_ = std::thread([this, waypoints, mission_id]() {
      execute_mission_thread(waypoints, mission_id);
    });
  }

  void publish_status(const std::string& mission_id, size_t current, size_t total,
                      const std::string& state) {
    std_msgs::msg::String msg;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      R"({"mission_id":"%s","current_waypoint":%zu,"total":%zu,"state":"%s"})",
      mission_id.c_str(), current, total, state.c_str());
    msg.data = buf;
    status_pub_->publish(msg);
  }

  void execute_mission_thread(
    const std::vector<std::tuple<double, double, double>>& waypoints,
    const std::string& mission_id)
  {
    RCLCPP_INFO(get_logger(), "Executing mission %s (%zu waypoints)",
                mission_id.c_str(), waypoints.size());

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
      publish_status(mission_id, 0, waypoints.size(), "failed");
      mission_running_.store(false);
      return;
    }

    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (mission_cancel_.load()) {
        RCLCPP_WARN(get_logger(), "Mission %s cancelled at waypoint %zu/%zu",
                    mission_id.c_str(), i + 1, waypoints.size());
        publish_status(mission_id, i, waypoints.size(), "cancelled");
        mission_running_.store(false);
        return;
      }

      auto [x, y, theta] = waypoints[i];
      publish_status(mission_id, i + 1, waypoints.size(), "navigating");
      RCLCPP_INFO(get_logger(), "  Goal %zu/%zu: (%.2f, %.2f)", i + 1, waypoints.size(), x, y);

      auto goal = NavigateToPose::Goal();
      goal.pose.header.frame_id = "map";
      goal.pose.header.stamp = this->now();
      goal.pose.pose.position.x = x;
      goal.pose.pose.position.y = y;
      goal.pose.pose.orientation.z = std::sin(theta / 2.0);
      goal.pose.pose.orientation.w = std::cos(theta / 2.0);

      auto goal_handle_future = nav_client_->async_send_goal(goal);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                              goal_handle_future, std::chrono::seconds(10)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to send goal %zu", i + 1);
        publish_status(mission_id, i + 1, waypoints.size(), "failed");
        mission_running_.store(false);
        return;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal %zu rejected", i + 1);
        publish_status(mission_id, i + 1, waypoints.size(), "failed");
        mission_running_.store(false);
        return;
      }

      auto result_future = nav_client_->async_get_result(goal_handle);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                              result_future, std::chrono::minutes(5)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Goal %zu timed out", i + 1);
        publish_status(mission_id, i + 1, waypoints.size(), "failed");
        mission_running_.store(false);
        return;
      }

      auto result = result_future.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(get_logger(), "Goal %zu failed", i + 1);
        publish_status(mission_id, i + 1, waypoints.size(), "failed");
        mission_running_.store(false);
        return;
      }

      RCLCPP_INFO(get_logger(), "  Goal %zu/%zu reached", i + 1, waypoints.size());
    }

    publish_status(mission_id, waypoints.size(), waypoints.size(), "completed");
    RCLCPP_INFO(get_logger(), "Mission %s completed", mission_id.c_str());
    mission_running_.store(false);
  }

  std::string missions_file_;
  double default_speed_;
  rclcpp::Service<agv_interfaces::srv::SaveWaypoint>::SharedPtr save_srv_;
  rclcpp::Service<agv_interfaces::srv::ListMissions>::SharedPtr list_srv_;
  rclcpp::Service<agv_interfaces::srv::ExecuteMission>::SharedPtr execute_srv_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Non-blocking execution
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  std::thread exec_thread_;
  std::atomic<bool> mission_running_{false};
  std::atomic<bool> mission_cancel_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManagerNode>());
  rclcpp::shutdown();
  return 0;
}
