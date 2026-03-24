#include <gtest/gtest.h>
#include <cmath>

namespace kinematics {

struct OdomDelta {
  double dx, dy, dtheta;
};

OdomDelta wheels_to_odom(double delta_left_turns, double delta_right_turns,
                          double wheel_radius, double track_width, double current_theta) {
  double dl = delta_left_turns * wheel_radius * 2.0 * M_PI;
  double dr = delta_right_turns * wheel_radius * 2.0 * M_PI;
  double ds = (dl + dr) / 2.0;
  double dtheta = (dr - dl) / track_width;
  double mid_theta = current_theta + dtheta / 2.0;
  return {ds * std::cos(mid_theta), ds * std::sin(mid_theta), dtheta};
}

}  // namespace kinematics

constexpr double WHEEL_R = 0.1;
constexpr double TRACK_W = 0.5;

TEST(OdomIntegration, StraightLine30Seconds) {
  double x = 0, y = 0, theta = 0;
  constexpr double dt = 0.02;  // 50 Hz
  constexpr double speed_turns_per_s = 1.0;
  constexpr int steps = 1500;  // 30 seconds

  for (int i = 0; i < steps; ++i) {
    double delta_turns = speed_turns_per_s * dt;
    auto d = kinematics::wheels_to_odom(delta_turns, delta_turns, WHEEL_R, TRACK_W, theta);
    x += d.dx;
    y += d.dy;
    theta += d.dtheta;
  }

  // Expected: x = speed * wheel_circumference * time = 1.0 * 0.2π * 30 ≈ 18.85 m
  double expected_x = speed_turns_per_s * WHEEL_R * 2.0 * M_PI * 30.0;
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
  double v_right = omega * (R + TRACK_W / 2.0);   // m/s

  // Convert to turns/s
  double left_turns_per_s  = v_left / (WHEEL_R * 2.0 * M_PI);
  double right_turns_per_s = v_right / (WHEEL_R * 2.0 * M_PI);

  // Full circle time = 2π / omega
  double circle_time = 2.0 * M_PI / omega;
  int steps = static_cast<int>(circle_time / dt);

  for (int i = 0; i < steps; ++i) {
    double dl = left_turns_per_s * dt;
    double dr = right_turns_per_s * dt;
    auto d = kinematics::wheels_to_odom(dl, dr, WHEEL_R, TRACK_W, theta);
    x += d.dx;
    y += d.dy;
    theta += d.dtheta;
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
    auto d = kinematics::wheels_to_odom(0.0, 0.0, WHEEL_R, TRACK_W, theta);
    x += d.dx;
    y += d.dy;
    theta += d.dtheta;
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
  double left_turns_per_s  = -v_lin / (WHEEL_R * 2.0 * M_PI);
  double right_turns_per_s =  v_lin / (WHEEL_R * 2.0 * M_PI);

  // Full rotation time = 2π / 1.0 rad/s
  int steps = static_cast<int>(2.0 * M_PI / dt);

  for (int i = 0; i < steps; ++i) {
    double dl = left_turns_per_s * dt;
    double dr = right_turns_per_s * dt;
    auto d = kinematics::wheels_to_odom(dl, dr, WHEEL_R, TRACK_W, theta);
    x += d.dx;
    y += d.dy;
    theta += d.dtheta;
  }

  // Should stay near origin but complete 2π rotation
  EXPECT_NEAR(x, 0.0, 0.05);
  EXPECT_NEAR(y, 0.0, 0.05);
  EXPECT_NEAR(theta, 2.0 * M_PI, 0.05);
}
