#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "agv_safety/cmd_vel_gate.hpp"

using namespace agv_safety;

TEST(GateLogic, PassesThroughWhenSafe) {
  GateInputs in;
  in.input_cmd.linear.x = 0.3;
  in.input_cmd.angular.z = 0.5;
  in.safety_ok = true;
  in.hardware_estop = false;
  in.max_linear = 0.5;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.5);
}

TEST(GateLogic, ZerosWhenNotSafe) {
  GateInputs in;
  in.input_cmd.linear.x = 0.3;
  in.input_cmd.angular.z = 0.5;
  in.safety_ok = false;
  in.max_linear = 0.5;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.0);
}

TEST(GateLogic, ZerosWhenHardwareEstop) {
  GateInputs in;
  in.input_cmd.linear.x = 0.3;
  in.safety_ok = true;
  in.hardware_estop = true;
  in.max_linear = 0.5;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
}

TEST(GateLogic, ClampsToMaxLinear) {
  GateInputs in;
  in.input_cmd.linear.x = 5.0;
  in.input_cmd.angular.z = 0.0;
  in.safety_ok = true;
  in.max_linear = 0.4;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.linear.x, 0.4);
}

TEST(GateLogic, ClampsNegativeLinear) {
  GateInputs in;
  in.input_cmd.linear.x = -5.0;
  in.safety_ok = true;
  in.max_linear = 0.4;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.linear.x, -0.4);
}

TEST(GateLogic, ClampsAngular) {
  GateInputs in;
  in.input_cmd.angular.z = 10.0;
  in.safety_ok = true;
  in.max_linear = 0.5;
  in.max_angular = 1.5;

  const auto out = apply_gate(in);
  EXPECT_DOUBLE_EQ(out.angular.z, 1.5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
