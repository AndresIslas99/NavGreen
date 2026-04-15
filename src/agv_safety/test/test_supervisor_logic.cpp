#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "agv_safety/safety_supervisor.hpp"

using namespace agv_safety;
using namespace std::chrono_literals;

class SupervisorLogic : public ::testing::Test {
 protected:
  rclcpp::Time t0_{1'000'000'000, 0, RCL_ROS_TIME};
};

TEST_F(SupervisorLogic, EmptyTopicsAreOk) {
  std::vector<MonitoredTopic> topics;
  const auto v = evaluate_topics(topics, t0_, t0_, false, 2000ms);
  EXPECT_TRUE(v.safety_ok);
  EXPECT_TRUE(v.silent_topics.empty());
}

TEST_F(SupervisorLogic, EstopForcesNotOk) {
  std::vector<MonitoredTopic> topics;
  const auto v = evaluate_topics(topics, t0_, t0_, true, 2000ms);
  EXPECT_FALSE(v.safety_ok);
  EXPECT_FALSE(v.alerts.empty());
  EXPECT_EQ(v.reason, "Software E-stop latched");
}

TEST_F(SupervisorLogic, FreshTopicIsOk) {
  std::vector<MonitoredTopic> topics;
  MonitoredTopic t;
  t.name = "/agv/wheel_odom";
  t.deadline = 200ms;
  t.last_seen = t0_;
  t.ever_seen = true;
  topics.push_back(t);

  const auto v = evaluate_topics(topics, t0_ + rclcpp::Duration(0, 100'000'000), t0_, false, 2000ms);
  EXPECT_TRUE(v.safety_ok);
}

TEST_F(SupervisorLogic, StaleTopicTriggersSilent) {
  std::vector<MonitoredTopic> topics;
  MonitoredTopic t;
  t.name = "/agv/wheel_odom";
  t.deadline = 200ms;
  t.last_seen = t0_;
  t.ever_seen = true;
  topics.push_back(t);

  const auto v = evaluate_topics(topics, t0_ + rclcpp::Duration(0, 500'000'000), t0_, false, 2000ms);
  EXPECT_FALSE(v.safety_ok);
  ASSERT_EQ(v.silent_topics.size(), 1u);
  EXPECT_EQ(v.silent_topics[0], "/agv/wheel_odom");
}

TEST_F(SupervisorLogic, NeverSeenInsideGraceIsTolerated) {
  std::vector<MonitoredTopic> topics;
  MonitoredTopic t;
  t.name = "/agv/scan";
  t.deadline = 200ms;
  t.ever_seen = false;
  topics.push_back(t);

  const auto v = evaluate_topics(topics, t0_ + rclcpp::Duration(0, 500'000'000), t0_, false, 2000ms);
  EXPECT_TRUE(v.safety_ok);
}

TEST_F(SupervisorLogic, NeverSeenAfterGraceIsSilent) {
  std::vector<MonitoredTopic> topics;
  MonitoredTopic t;
  t.name = "/agv/scan";
  t.deadline = 200ms;
  t.ever_seen = false;
  topics.push_back(t);

  const auto v = evaluate_topics(topics, t0_ + rclcpp::Duration(3, 0), t0_, false, 2000ms);
  EXPECT_FALSE(v.safety_ok);
  ASSERT_EQ(v.silent_topics.size(), 1u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
