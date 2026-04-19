#include <gtest/gtest.h>
#include <cmath>

// Differential drive kinematics functions (extracted for testability)
namespace kinematics {

struct WheelVelocities {
  double left;   // turns/s
  double right;  // turns/s
};

WheelVelocities cmd_vel_to_wheels(double linear_x, double angular_z,
                                   double wheel_radius, double track_width) {
  double v_left  = (linear_x - angular_z * track_width / 2.0) / (wheel_radius * 2.0 * M_PI);
  double v_right = (linear_x + angular_z * track_width / 2.0) / (wheel_radius * 2.0 * M_PI);
  return {v_left, v_right};
}

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
constexpr double EPS = 1e-6;

TEST(Kinematics, StraightForward) {
  auto w = kinematics::cmd_vel_to_wheels(1.0, 0.0, WHEEL_R, TRACK_W);
  EXPECT_NEAR(w.left, w.right, EPS);
  EXPECT_GT(w.left, 0.0);
}

TEST(Kinematics, StraightBackward) {
  auto w = kinematics::cmd_vel_to_wheels(-1.0, 0.0, WHEEL_R, TRACK_W);
  EXPECT_NEAR(w.left, w.right, EPS);
  EXPECT_LT(w.left, 0.0);
}

TEST(Kinematics, PureRotation) {
  auto w = kinematics::cmd_vel_to_wheels(0.0, 1.0, WHEEL_R, TRACK_W);
  EXPECT_NEAR(w.left, -w.right, EPS);
  EXPECT_LT(w.left, 0.0);
  EXPECT_GT(w.right, 0.0);
}

TEST(Kinematics, ZeroInput) {
  auto w = kinematics::cmd_vel_to_wheels(0.0, 0.0, WHEEL_R, TRACK_W);
  EXPECT_NEAR(w.left, 0.0, EPS);
  EXPECT_NEAR(w.right, 0.0, EPS);
}

TEST(Kinematics, ArcMotion) {
  auto w = kinematics::cmd_vel_to_wheels(1.0, 0.5, WHEEL_R, TRACK_W);
  EXPECT_GT(w.right, w.left);  // right wheel faster in left turn
  EXPECT_GT(w.left, 0.0);      // both forward
}

TEST(Kinematics, OdomStraight) {
  auto d = kinematics::wheels_to_odom(1.0, 1.0, WHEEL_R, TRACK_W, 0.0);
  EXPECT_GT(d.dx, 0.0);
  EXPECT_NEAR(d.dy, 0.0, EPS);
  EXPECT_NEAR(d.dtheta, 0.0, EPS);
}

TEST(Kinematics, OdomPureRotation) {
  auto d = kinematics::wheels_to_odom(-1.0, 1.0, WHEEL_R, TRACK_W, 0.0);
  EXPECT_NEAR(d.dx, 0.0, 0.01);  // small arc approximation
  EXPECT_GT(d.dtheta, 0.0);       // positive rotation (CCW)
}

TEST(Kinematics, OdomReversible) {
  // Forward then backward should return near origin
  double x = 0, y = 0, theta = 0;
  auto d1 = kinematics::wheels_to_odom(1.0, 1.0, WHEEL_R, TRACK_W, theta);
  x += d1.dx; y += d1.dy; theta += d1.dtheta;
  auto d2 = kinematics::wheels_to_odom(-1.0, -1.0, WHEEL_R, TRACK_W, theta);
  x += d2.dx; y += d2.dy; theta += d2.dtheta;
  EXPECT_NEAR(x, 0.0, EPS);
  EXPECT_NEAR(y, 0.0, EPS);
  EXPECT_NEAR(theta, 0.0, EPS);
}
