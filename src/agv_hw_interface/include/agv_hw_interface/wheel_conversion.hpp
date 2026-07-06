#pragma once

// Wheel <-> motor unit conversions shared between AgvDiffDriveSystem and its
// unit tests, so the tests exercise the exact arithmetic the plugin runs.
//
// Units: wheel angles/velocities in rad / rad/s (ros2_control convention),
// motor positions/velocities in turns / turns/s (ODrive convention),
// gear_ratio = motor turns per wheel turn.

#include <cmath>

namespace agv_hw_interface {

inline double wheel_rad_to_motor_turns(double wheel_rad, double gear_ratio, bool invert) {
  double wheel_turns = wheel_rad / (2.0 * M_PI);
  if (invert) wheel_turns = -wheel_turns;
  return wheel_turns * gear_ratio;
}

inline double motor_turns_to_wheel_rad(double motor_turns, double gear_ratio, bool invert) {
  double wheel_turns = motor_turns / gear_ratio;
  if (invert) wheel_turns = -wheel_turns;
  return wheel_turns * 2.0 * M_PI;
}

}  // namespace agv_hw_interface
