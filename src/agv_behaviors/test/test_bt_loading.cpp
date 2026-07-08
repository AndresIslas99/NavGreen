#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace fs = std::filesystem;

// Mirror of the executor's registration: same registration name and ports
// as NavigateToPoseAction in behavior_executor_node.cpp, minus ROS. Keeps
// this test independent of a running ROS graph while still rejecting trees
// that reference unregistered node types or undeclared ports. If the
// executor's providedPorts() changes, update this stub to match.
class FakeNavigateToPose : public BT::SyncActionNode {
public:
  FakeNavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("theta", 0.0, "Goal orientation"),
      BT::InputPort<std::string>("server_name", "navigate_to_pose", "Action server name"),
    };
  }

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
};

TEST(BehaviorTreeLoading, FactoryCreation) {
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(BT::BehaviorTreeFactory());
}

TEST(BehaviorTreeLoading, XMLParseSimpleTree) {
  BT::BehaviorTreeFactory factory;

  // A minimal valid BT XML
  const char* xml = R"(
    <root main_tree_to_execute="Test">
      <BehaviorTree ID="Test">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

// Every XML shipped in trees/ must load through a factory with the same
// node registrations the executor has. This is the regression gate that
// previously let unloadable trees (waypoint_patrol.xml,
// navigate_with_recovery.xml — node types never registered anywhere) ship
// for months.
TEST(BehaviorTreeLoading, EveryShippedTreeLoads) {
  const fs::path trees_dir{AGV_BEHAVIORS_TREES_DIR};
  ASSERT_TRUE(fs::is_directory(trees_dir)) << trees_dir;

  size_t xml_count = 0;
  for (const auto& entry : fs::directory_iterator(trees_dir)) {
    if (entry.path().extension() != ".xml") continue;
    ++xml_count;
    // Fresh factory per file so one bad tree cannot mask another.
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<FakeNavigateToPose>("NavigateToPose");
    try {
      auto tree = factory.createTreeFromFile(entry.path().string());
      (void)tree;
    } catch (const std::exception& e) {
      ADD_FAILURE() << entry.path() << " failed to load: " << e.what();
    }
  }
  // At least the default tree (single_waypoint.xml) must ship.
  EXPECT_GE(xml_count, 1u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
