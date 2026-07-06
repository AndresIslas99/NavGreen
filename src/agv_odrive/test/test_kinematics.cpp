#include <gtest/gtest.h>
#include <cmath>

// Exercises the exact kinematics the production node runs — see
// include/agv_odrive/kinematics.hpp (shared by odrive_can_node.cpp).
#include "agv_odrive/kinematics.hpp"

using agv_odrive::kinematics::cmd_vel_to_wheels;
using agv_odrive::kinematics::motor_turns_to_meters;
using agv_odrive::kinematics::wheel_deltas_to_odom;

constexpr double WHEEL_R = 0.1;
constexpr double TRACK_W = 0.5;
constexpr double GEAR = 1.0;  // direct drive keeps the geometric tests readable
constexpr double EPS = 1e-6;

TEST(Kinematics, StraightForward) {
  auto w = cmd_vel_to_wheels(1.0, 0.0, WHEEL_R, TRACK_W, GEAR);
  EXPECT_NEAR(w.left, w.right, EPS);
  EXPECT_GT(w.left, 0.0);
}

TEST(Kinematics, StraightBackward) {
  auto w = cmd_vel_to_wheels(-1.0, 0.0, WHEEL_R, TRACK_W, GEAR);
  EXPECT_NEAR(w.left, w.right, EPS);
  EXPECT_LT(w.left, 0.0);
}

TEST(Kinematics, PureRotation) {
  auto w = cmd_vel_to_wheels(0.0, 1.0, WHEEL_R, TRACK_W, GEAR);
  EXPECT_NEAR(w.left, -w.right, EPS);
  EXPECT_LT(w.left, 0.0);
  EXPECT_GT(w.right, 0.0);
}

TEST(Kinematics, ZeroInput) {
  auto w = cmd_vel_to_wheels(0.0, 0.0, WHEEL_R, TRACK_W, GEAR);
  EXPECT_NEAR(w.left, 0.0, EPS);
  EXPECT_NEAR(w.right, 0.0, EPS);
}

TEST(Kinematics, ArcMotion) {
  auto w = cmd_vel_to_wheels(1.0, 0.5, WHEEL_R, TRACK_W, GEAR);
  EXPECT_GT(w.right, w.left);  // right wheel faster in left turn
  EXPECT_GT(w.left, 0.0);      // both forward
}

TEST(Kinematics, GearRatioScalesMotorVelocity) {
  // A 10:1 gearbox needs 10x the motor turns for the same body velocity.
  auto direct = cmd_vel_to_wheels(0.5, 0.0, WHEEL_R, TRACK_W, 1.0);
  auto geared = cmd_vel_to_wheels(0.5, 0.0, WHEEL_R, TRACK_W, 10.0);
  EXPECT_NEAR(geared.left, 10.0 * direct.left, EPS);
  EXPECT_NEAR(geared.right, 10.0 * direct.right, EPS);
}

TEST(Kinematics, MotorTurnsToMetersAppliesGearRatio) {
  // 10 motor turns through a 10:1 gearbox = 1 wheel turn = 2*pi*r meters.
  EXPECT_NEAR(motor_turns_to_meters(10.0, 10.0, WHEEL_R), 2.0 * M_PI * WHEEL_R, EPS);
}

TEST(Kinematics, CommandFeedbackRoundTrip) {
  // cmd_vel -> motor turns/s -> back to m/s must reproduce the command.
  const double linear = 0.4;
  auto w = cmd_vel_to_wheels(linear, 0.0, WHEEL_R, TRACK_W, 10.0);
  EXPECT_NEAR(motor_turns_to_meters(w.left, 10.0, WHEEL_R), linear, EPS);
  EXPECT_NEAR(motor_turns_to_meters(w.right, 10.0, WHEEL_R), linear, EPS);
}

TEST(Kinematics, OdomStraight) {
  double d_m = motor_turns_to_meters(1.0, GEAR, WHEEL_R);
  auto d = wheel_deltas_to_odom(d_m, d_m, TRACK_W, 0.0);
  EXPECT_GT(d.dx, 0.0);
  EXPECT_NEAR(d.dy, 0.0, EPS);
  EXPECT_NEAR(d.dtheta, 0.0, EPS);
}

TEST(Kinematics, OdomPureRotation) {
  double d_m = motor_turns_to_meters(1.0, GEAR, WHEEL_R);
  auto d = wheel_deltas_to_odom(-d_m, d_m, TRACK_W, 0.0);
  EXPECT_NEAR(d.dx, 0.0, 0.01);  // small arc approximation
  EXPECT_GT(d.dtheta, 0.0);       // positive rotation (CCW)
}

TEST(Kinematics, OdomReversible) {
  // Forward then backward should return near origin
  double x = 0, y = 0, theta = 0;
  double d_m = motor_turns_to_meters(1.0, GEAR, WHEEL_R);
  auto d1 = wheel_deltas_to_odom(d_m, d_m, TRACK_W, theta);
  x += d1.dx; y += d1.dy; theta += d1.dtheta;
  auto d2 = wheel_deltas_to_odom(-d_m, -d_m, TRACK_W, theta);
  x += d2.dx; y += d2.dy; theta += d2.dtheta;
  EXPECT_NEAR(x, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);
  EXPECT_NEAR(theta, 0.0, EPS);
}
