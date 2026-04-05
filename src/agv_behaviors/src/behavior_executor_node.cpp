// agv_behaviors — C++17 behavior tree executor for mission execution
//
// Loads behavior tree XML files and executes them against Nav2's
// navigate_to_pose action server. Provides a service interface for
// triggering mission execution from the dashboard.
//
// Custom BT node "NavigateToPose" wraps the Nav2 action client directly,
// avoiding a hard dependency on nav2_behavior_tree package.

#include <atomic>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <agv_interfaces/srv/execute_mission.hpp>
#include <std_msgs/msg/string.hpp>

namespace fs = std::filesystem;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// ── Custom BT action node that wraps Nav2's navigate_to_pose ──

class NavigateToPoseAction : public BT::StatefulActionNode {
public:
  NavigateToPoseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("theta", 0.0, "Goal orientation"),
      BT::InputPort<std::string>("server_name", "navigate_to_pose", "Action server name"),
    };
  }

  BT::NodeStatus onStart() override {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    auto nav_client = config().blackboard->get<
      rclcpp_action::Client<NavigateToPose>::SharedPtr>("nav_client");

    double x = 0, y = 0, theta = 0;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(theta / 2.0);
    goal.pose.pose.orientation.w = std::cos(theta / 2.0);

    RCLCPP_INFO(node->get_logger(), "BT NavigateToPose: sending goal (%.2f, %.2f)", x, y);

    auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_options.result_callback = [this](const auto& result) {
      goal_done_ = true;
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

    goal_done_ = false;
    goal_succeeded_ = false;
    nav_client->async_send_goal(goal, send_options);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!goal_done_) return BT::NodeStatus::RUNNING;
    return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override {
    // Cancel navigation on halt (preemption)
    auto nav_client = config().blackboard->get<
      rclcpp_action::Client<NavigateToPose>::SharedPtr>("nav_client");
    nav_client->async_cancel_all_goals();
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "BT NavigateToPose: goal halted/cancelled");
  }

private:
  std::atomic<bool> goal_done_{false};
  std::atomic<bool> goal_succeeded_{false};
};

// ── Main executor node ──

class BehaviorExecutorNode : public rclcpp::Node {
public:
  BehaviorExecutorNode() : Node("behavior_executor") {
    this->declare_parameter("trees_dir", "");
    this->declare_parameter("default_tree", "single_waypoint.xml");

    trees_dir_ = this->get_parameter("trees_dir").as_string();
    default_tree_ = this->get_parameter("default_tree").as_string();

    // Register our custom BT action node
    factory_.registerNodeType<NavigateToPoseAction>("NavigateToPose");

    execute_srv_ = this->create_service<agv_interfaces::srv::ExecuteMission>(
      "behavior_executor/execute",
      std::bind(&BehaviorExecutorNode::on_execute, this,
                std::placeholders::_1, std::placeholders::_2));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("behavior_executor/status", 10);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(get_logger(), "Behavior executor ready, trees_dir=%s, default=%s",
                trees_dir_.c_str(), default_tree_.c_str());
  }

  ~BehaviorExecutorNode() override {
    cancel_requested_ = true;
    if (exec_thread_.joinable()) exec_thread_.join();
  }

private:
  void on_execute(
    const agv_interfaces::srv::ExecuteMission::Request::SharedPtr req,
    agv_interfaces::srv::ExecuteMission::Response::SharedPtr res)
  {
    if (running_.load()) {
      res->success = false;
      res->message = "A behavior tree is already running";
      return;
    }

    // Determine which tree to load
    std::string tree_file = default_tree_;
    // Convention: if mission_id ends with .xml, use it as tree name
    if (req->mission_id.size() > 4 &&
        req->mission_id.substr(req->mission_id.size() - 4) == ".xml") {
      tree_file = req->mission_id;
    }

    std::string tree_path = trees_dir_ + "/" + tree_file;
    if (!fs::exists(tree_path)) {
      res->success = false;
      res->message = "Tree file not found: " + tree_path;
      return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      res->success = false;
      res->message = "Nav2 action server not available";
      return;
    }

    res->success = true;
    res->message = "Behavior tree started: " + tree_file;

    // Launch BT execution in background thread
    if (exec_thread_.joinable()) exec_thread_.join();
    cancel_requested_ = false;
    running_.store(true);

    exec_thread_ = std::thread([this, tree_path, mission_id = req->mission_id]() {
      run_tree(tree_path, mission_id);
    });
  }

  void run_tree(const std::string& tree_path, const std::string& mission_id) {
    RCLCPP_INFO(get_logger(), "Loading BT from %s", tree_path.c_str());

    try {
      auto tree = factory_.createTreeFromFile(tree_path);

      // Populate blackboard with shared resources
      tree.rootBlackboard()->set("node", shared_from_this());
      tree.rootBlackboard()->set("nav_client", nav_client_);

      publish_status(mission_id, "running");

      // Tick loop at 10Hz
      auto period = std::chrono::milliseconds(100);
      BT::NodeStatus status = BT::NodeStatus::RUNNING;

      while (status == BT::NodeStatus::RUNNING && !cancel_requested_) {
        status = tree.tickRoot();
        rclcpp::sleep_for(period);
      }

      if (cancel_requested_) {
        tree.haltTree();
        publish_status(mission_id, "cancelled");
        RCLCPP_WARN(get_logger(), "BT execution cancelled for %s", mission_id.c_str());
      } else if (status == BT::NodeStatus::SUCCESS) {
        publish_status(mission_id, "completed");
        RCLCPP_INFO(get_logger(), "BT execution succeeded for %s", mission_id.c_str());
      } else {
        publish_status(mission_id, "failed");
        RCLCPP_ERROR(get_logger(), "BT execution failed for %s", mission_id.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "BT execution error: %s", e.what());
      publish_status(mission_id, "error");
    }

    running_.store(false);
  }

  void publish_status(const std::string& mission_id, const std::string& state) {
    std_msgs::msg::String msg;
    msg.data = "{\"mission_id\":\"" + mission_id + "\",\"state\":\"" + state + "\"}";
    status_pub_->publish(msg);
  }

  std::string trees_dir_;
  std::string default_tree_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::Service<agv_interfaces::srv::ExecuteMission>::SharedPtr execute_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  std::thread exec_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> cancel_requested_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
