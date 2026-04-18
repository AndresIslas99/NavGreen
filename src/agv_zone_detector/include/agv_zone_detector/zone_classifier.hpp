#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace agv_zone_detector {

struct RailSection {
  double x_start;
  double x_end;
  std::string name;
};

struct ClassifyResult {
  std::string zone;           // "corridor_west" | "corridor_east" | "gap" |
                              // "rail_aisle_0" | "rail_aisle_p22" | "rail_aisle_m22" |
                              // "rail_aisle_p44" | "rail_aisle_m44" | "unknown"
  std::string section;        // "REAR" | "FRONT" | "GAP" | "OUTSIDE"
  double aisle_y_center;      // If in rail: matching aisle y center; else NaN.
  double rail_offset_lat;     // Signed lateral offset from aisle center (m). NaN if not in rail.
  double rail_yaw_error;      // Yaw vs rail X-axis, wrapped to [-pi, pi]. NaN if not in rail.
  double confidence;          // 0..1
};

// Pure classification function, no ROS deps. Easy to unit-test.
//
// Greenhouse geometry (operator-confirmed 2026-04-18):
//   REAR rail section: x ∈ [-16.5, 3.5]
//   GAP (rail-free):   x ∈ ( 3.5,  7.5]
//   FRONT rail section: x ∈ [7.5, 27.5]
//   Outside: x < -16.5 or x > 27.5 (if map extends)
//
// Each section has 5 aisles at y ∈ {-4.4, -2.2, 0, +2.2, +4.4}.
// Aisle "in range": |y - y_aisle| < aisle_half_width (default 0.35 m,
// chosen so the 0.45-m rail pair centered at y_aisle is always detected).
ClassifyResult classify(double x, double y, double yaw,
                        double aisle_half_width = 0.35);

// Normalize a yaw angle to [-pi, pi).
inline double wrap_to_pi(double a) {
  while (a >=  M_PI) a -= 2.0 * M_PI;
  while (a <  -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace agv_zone_detector
