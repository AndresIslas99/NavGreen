#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
