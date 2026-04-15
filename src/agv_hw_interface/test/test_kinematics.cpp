#include <gtest/gtest.h>

#include <cmath>

// Pure-logic tests for the radians<->turns conversion this plugin performs.
// We do not instantiate the SystemInterface here (that requires a fully
// loaded HardwareInfo); we just verify the gear-ratio and inversion math
// match what diff_drive_controller will see.

namespace {
constexpr double TWO_PI = 2.0 * M_PI;

// Mirror of AgvDiffDriveSystem::send_velocity arithmetic.
double wheel_rad_per_s_to_motor_turns_per_s(double wheel_rad_per_s,
                                            double gear_ratio,
                                            bool invert) {
  double wheel_turns_per_s = wheel_rad_per_s / TWO_PI;
  if (invert) wheel_turns_per_s = -wheel_turns_per_s;
  return wheel_turns_per_s * gear_ratio;
}

// Mirror of AgvDiffDriveSystem::process_encoder_frame arithmetic.
double motor_turns_to_wheel_rad(double motor_turns,
                                double gear_ratio,
                                bool invert) {
  double wheel_turns = motor_turns / gear_ratio;
  if (invert) wheel_turns = -wheel_turns;
  return wheel_turns * TWO_PI;
}
}  // namespace

TEST(Kinematics, ZeroIsZero) {
  EXPECT_DOUBLE_EQ(wheel_rad_per_s_to_motor_turns_per_s(0.0, 10.0, false), 0.0);
  EXPECT_DOUBLE_EQ(wheel_rad_per_s_to_motor_turns_per_s(0.0, 10.0, true),  0.0);
}

TEST(Kinematics, OneRadPerSecAtTenRatio) {
  // 1 rad/s = (1 / 2pi) wheel turns/s -> *10 = 10/(2pi) motor turns/s
  const double expected = 10.0 / TWO_PI;
  EXPECT_NEAR(wheel_rad_per_s_to_motor_turns_per_s(1.0, 10.0, false), expected, 1e-9);
}

TEST(Kinematics, InvertNegates) {
  EXPECT_NEAR(wheel_rad_per_s_to_motor_turns_per_s(2.0, 10.0, true),
              -wheel_rad_per_s_to_motor_turns_per_s(2.0, 10.0, false),
              1e-9);
}

TEST(Kinematics, RoundTripCommandToFeedback) {
  // Send 0.5 rad/s, then read back the motor position equivalent.
  const double cmd_rad_per_s = 0.5;
  const double gear = 10.0;
  const double dt_s = 1.0;

  const double motor_turns_per_s =
      wheel_rad_per_s_to_motor_turns_per_s(cmd_rad_per_s, gear, false);
  const double motor_turns_after_1s = motor_turns_per_s * dt_s;
  const double wheel_rad_after_1s = motor_turns_to_wheel_rad(motor_turns_after_1s, gear, false);

  EXPECT_NEAR(wheel_rad_after_1s, cmd_rad_per_s * dt_s, 1e-9);
}

TEST(Kinematics, InvertedRoundTrip) {
  const double cmd_rad_per_s = 0.5;
  const double gear = 10.0;

  const double motor_turns_per_s =
      wheel_rad_per_s_to_motor_turns_per_s(cmd_rad_per_s, gear, true);
  // After invert, motor turns are negated.
  EXPECT_LT(motor_turns_per_s, 0.0);

  // Reading back with the same invert flag must restore the original sign.
  const double wheel_rad_per_s = motor_turns_to_wheel_rad(motor_turns_per_s, gear, true);
  EXPECT_NEAR(wheel_rad_per_s, cmd_rad_per_s, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
