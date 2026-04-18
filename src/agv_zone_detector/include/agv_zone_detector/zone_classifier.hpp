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
                              // "rail_approach_front" | "rail_approach_rear" |
                              // "rail_aisle_0" | "rail_aisle_p22" | "rail_aisle_m22" |
                              // "rail_aisle_p44" | "rail_aisle_m44" | "unknown"
  std::string section;        // "REAR" | "GAP" | "FRONT" | "OUTSIDE" |
                              // "APPROACH_REAR" | "APPROACH_FRONT"
  double aisle_y_center;      // If in rail or rail_approach: matching aisle y; else NaN.
  double rail_offset_lat;     // Signed lateral offset from aisle center (m). NaN if not in rail/approach.
  double rail_yaw_error;      // Yaw vs rail X-axis, wrapped to [-pi, pi]. NaN if not in rail/approach.
  double confidence;          // 0..1
  int    approach_tag_id;     // Floor AprilTag ID at the approach entry; -1 if zone is not an approach.
};

// Pure classification function, no ROS deps. Easy to unit-test.
//
// Greenhouse geometry (operator-confirmed 2026-04-18):
//   REAR rail section:   x ∈ [-16.5, 3.5]
//   APPROACH_REAR strip: x ∈ [ 4.0,  4.5)  — RAIL_APPROACH to REAR entry
//   GAP (rail-free):     x ∈ ( 3.5,  7.5)  — CORRIDOR_NAV zone
//   APPROACH_FRONT strip: x ∈ ( 6.5,  7.0]  — RAIL_APPROACH to FRONT entry
//   FRONT rail section:  x ∈ [ 7.5, 27.5]
//   Outside: x < -16.5 or x > 27.5 (if map extends)
//
// The two approach strips overlap with GAP by design — they are the
// "imminent rail entry" sub-zone within the gap where the mode_arbiter
// should hand off from Nav2 to the AprilTag-guided rail_approach. Floor
// AprilTags sit at x=4.0 (REAR, IDs 33/34/35/36/37 for aisles 1-5) and
// x=7.0 (FRONT, IDs 2/3/4/12/13 for aisles 1-5).
//
// Each section has 5 aisles at y ∈ {-4.4, -2.2, 0, +2.2, +4.4}.
// Aisle "in range": |y - y_aisle| < aisle_half_width (default 0.35 m,
// chosen so the 0.45-m rail pair centered at y_aisle is always detected).
ClassifyResult classify(double x, double y, double yaw,
                        double aisle_half_width = 0.35);

// Floor AprilTag IDs by aisle index (0..4 = y ∈ {-4.4, -2.2, 0, +2.2, +4.4}).
// Baked in agv-greenhouse-sim USD 2026-04-18.
// REAR entry (x=4.00, facing +Z): IDs 33, 34, 35, 36, 37.
// FRONT entry (x=7.00, facing +Z): IDs 2, 3, 4, 12, 13.
inline int tag_id_for_rear_approach(int aisle_idx) {
  constexpr std::array<int, 5> ids = {33, 34, 35, 36, 37};
  return (aisle_idx >= 0 && aisle_idx < 5) ? ids[aisle_idx] : -1;
}
inline int tag_id_for_front_approach(int aisle_idx) {
  constexpr std::array<int, 5> ids = {2, 3, 4, 12, 13};
  return (aisle_idx >= 0 && aisle_idx < 5) ? ids[aisle_idx] : -1;
}

// Normalize a yaw angle to [-pi, pi).
inline double wrap_to_pi(double a) {
  while (a >=  M_PI) a -= 2.0 * M_PI;
  while (a <  -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace agv_zone_detector
