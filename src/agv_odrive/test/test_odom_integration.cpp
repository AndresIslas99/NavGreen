#include <gtest/gtest.h>
#include <cmath>

// Long-horizon odometry integration tests running the exact forward
// kinematics the production node uses — see include/agv_odrive/kinematics.hpp.
#include "agv_odrive/kinematics.hpp"

using agv_odrive::kinematics::motor_turns_to_meters;
using agv_odrive::kinematics::wheel_deltas_to_odom;

constexpr double WHEEL_R = 0.1;
constexpr double TRACK_W = 0.5;
constexpr double GEAR = 10.0;  // production uses a 10:1 planetary gearbox

namespace {

// Integrate one step from per-wheel motor-turn deltas, as the node does.
void integrate(double dl_motor_turns, double dr_motor_turns,
               double& x, double& y, double& theta) {
  const double dl_m = motor_turns_to_meters(dl_motor_turns, GEAR, WHEEL_R);
  const double dr_m = motor_turns_to_meters(dr_motor_turns, GEAR, WHEEL_R);
  const auto d = wheel_deltas_to_odom(dl_m, dr_m, TRACK_W, theta);
  x += d.dx;
  y += d.dy;
  theta += d.dtheta;
}

// m/s at the wheel rim -> motor turns/s (inverse of motor_turns_to_meters).
double mps_to_motor_turns_per_s(double mps) {
  return mps / (WHEEL_R * 2.0 * M_PI) * GEAR;
}

}  // namespace

TEST(OdomIntegration, StraightLine30Seconds) {
  double x = 0, y = 0, theta = 0;
  constexpr double dt = 0.02;  // 50 Hz
  constexpr double motor_speed_turns_per_s = 10.0;  // = 1 wheel turn/s at 10:1
  constexpr int steps = 1500;  // 30 seconds

  for (int i = 0; i < steps; ++i) {
    double delta_turns = motor_speed_turns_per_s * dt;
    integrate(delta_turns, delta_turns, x, y, theta);
  }

  // Expected: x = wheel_turns_per_s * wheel_circumference * time
  //             = 1.0 * 0.2π * 30 ≈ 18.85 m
  double expected_x = (motor_speed_turns_per_s / GEAR) * WHEEL_R * 2.0 * M_PI * 30.0;
  EXPECT_NEAR(x, expected_x, 0.001);
  EXPECT_NEAR(y, 0.0, 0.001);
  EXPECT_NEAR(theta, 0.0, 0.001);
}

TEST(OdomIntegration, FullCircle) {
  double x = 0, y = 0, theta = 0;
  constexpr double dt = 0.02;

  // To make a circle: different wheel speeds
  // For a circle of radius R: v_left = omega*(R - track/2), v_right = omega*(R + track/2)
  // With R=1.0m, omega=0.5 rad/s:
  constexpr double R = 1.0;
  constexpr double omega = 0.5;
  double v_left  = omega * (R - TRACK_W / 2.0);  // m/s
  double v_right = omega * (R + TRACK_W / 2.0);  // m/s

  double left_turns_per_s  = mps_to_motor_turns_per_s(v_left);
  double right_turns_per_s = mps_to_motor_turns_per_s(v_right);

  // Full circle time = 2π / omega
  double circle_time = 2.0 * M_PI / omega;
  int steps = static_cast<int>(circle_time / dt);

  for (int i = 0; i < steps; ++i) {
    integrate(left_turns_per_s * dt, right_turns_per_s * dt, x, y, theta);
  }

  // Should return near origin after full circle
  EXPECT_NEAR(x, 0.0, 0.05);
  EXPECT_NEAR(y, 0.0, 0.05);
  // theta should be ≈ 2π (one full circle). fmod(2π, 2π) ≈ 0 or ≈ 2π due to float
  double wrapped = std::fmod(theta, 2.0 * M_PI);
  if (wrapped > M_PI) wrapped -= 2.0 * M_PI;
  EXPECT_NEAR(wrapped, 0.0, 0.05);
}

TEST(OdomIntegration, StationaryNoMovement) {
  double x = 0, y = 0, theta = 0;
  for (int i = 0; i < 100; ++i) {
    integrate(0.0, 0.0, x, y, theta);
  }
  EXPECT_NEAR(x, 0.0, 1e-10);
  EXPECT_NEAR(y, 0.0, 1e-10);
  EXPECT_NEAR(theta, 0.0, 1e-10);
}

TEST(OdomIntegration, Rotation360) {
  double x = 0, y = 0, theta = 0;
  constexpr double dt = 0.02;

  // Pure rotation: left backward, right forward
  // Angular velocity = (v_right - v_left) / track_width
  // For omega = 1 rad/s: v_right = track/2, v_left = -track/2
  double v_lin = TRACK_W / 2.0;  // m/s per wheel
  double left_turns_per_s  = mps_to_motor_turns_per_s(-v_lin);
  double right_turns_per_s = mps_to_motor_turns_per_s(v_lin);

  // Full rotation time = 2π / 1.0 rad/s
  int steps = static_cast<int>(2.0 * M_PI / dt);

  for (int i = 0; i < steps; ++i) {
    integrate(left_turns_per_s * dt, right_turns_per_s * dt, x, y, theta);
  }

  // Should stay near origin but complete 2π rotation
  EXPECT_NEAR(x, 0.0, 0.05);
  EXPECT_NEAR(y, 0.0, 0.05);
  EXPECT_NEAR(theta, 2.0 * M_PI, 0.05);
}
