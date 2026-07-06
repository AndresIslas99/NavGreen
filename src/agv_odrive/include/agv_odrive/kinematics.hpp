#pragma once

// Differential-drive kinematics shared between the production node
// (odrive_can_node.cpp) and its unit tests. Header-only so the tests
// exercise exactly the arithmetic the robot runs.
//
// Units and conventions:
//   linear_x      m/s     (+x forward)
//   angular_z     rad/s   (+z counter-clockwise)
//   wheel_radius  m
//   track_width   m       (effective wheel-to-wheel distance)
//   gear_ratio    motor turns per wheel turn (10.0 for the 10:1 planetary)
//   motor turns   ODrive Pos_Estimate / Vel_Estimate units

#include <cmath>

namespace agv_odrive {
namespace kinematics {

struct WheelVelocities {
  double left;   // motor turns/s
  double right;  // motor turns/s
};

// Inverse kinematics: body twist -> per-wheel motor velocity.
inline WheelVelocities cmd_vel_to_wheels(double linear_x, double angular_z,
                                         double wheel_radius, double track_width,
                                         double gear_ratio) {
  const double left =
      (linear_x - angular_z * track_width / 2.0) / (wheel_radius * 2.0 * M_PI) * gear_ratio;
  const double right =
      (linear_x + angular_z * track_width / 2.0) / (wheel_radius * 2.0 * M_PI) * gear_ratio;
  return {left, right};
}

// Motor turns (delta or turns/s) -> meters along the ground (or m/s).
inline double motor_turns_to_meters(double motor_turns, double gear_ratio,
                                    double wheel_radius) {
  return motor_turns / gear_ratio * wheel_radius * 2.0 * M_PI;
}

struct OdomDelta {
  double dx;      // m, odom frame
  double dy;      // m, odom frame
  double dtheta;  // rad
};

// Forward kinematics: per-wheel ground travel (meters) -> pose increment,
// using mid-angle integration (more accurate than Euler on arcs).
inline OdomDelta wheel_deltas_to_odom(double delta_left_m, double delta_right_m,
                                      double track_width, double current_theta) {
  const double ds = (delta_left_m + delta_right_m) / 2.0;
  const double dtheta = (delta_right_m - delta_left_m) / track_width;
  const double mid_theta = current_theta + dtheta / 2.0;
  return {ds * std::cos(mid_theta), ds * std::sin(mid_theta), dtheta};
}

}  // namespace kinematics
}  // namespace agv_odrive
