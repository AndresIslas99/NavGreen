#pragma once

#include "agv_zone_detector/zone_classifier.hpp"

namespace agv_zone_detector {

inline ClassifyResult classify(double x, double y, double yaw,
                               double aisle_half_width) {
  ClassifyResult r;
  r.aisle_y_center = std::nan("");
  r.rail_offset_lat = std::nan("");
  r.rail_yaw_error = std::nan("");

  // Section lookup.
  // REAR  x ∈ [-16.5,  3.5]
  // GAP   x ∈ ( 3.5,   7.5)
  // FRONT x ∈ [ 7.5,  27.5]
  constexpr double REAR_X_START  = -16.5;
  constexpr double REAR_X_END    =   3.5;
  constexpr double FRONT_X_START =   7.5;
  constexpr double FRONT_X_END   =  27.5;

  bool in_rear  = (x >= REAR_X_START  && x <= REAR_X_END);
  bool in_front = (x >= FRONT_X_START && x <= FRONT_X_END);
  bool in_gap   = (x >  REAR_X_END    && x <  FRONT_X_START);

  if (in_rear) {
    r.section = "REAR";
  } else if (in_front) {
    r.section = "FRONT";
  } else if (in_gap) {
    r.section = "GAP";
    r.zone = "gap";
    r.confidence = 1.0;
    return r;
  } else {
    r.section = "OUTSIDE";
    r.zone = (x < REAR_X_START) ? "corridor_west" : "corridor_east";
    r.confidence = 1.0;
    return r;
  }

  // In a rail section — determine which aisle (if any).
  constexpr std::array<double, 5> AISLE_CENTERS = {-4.4, -2.2, 0.0, 2.2, 4.4};
  constexpr std::array<const char *, 5> AISLE_NAMES = {
    "rail_aisle_m44", "rail_aisle_m22", "rail_aisle_0",
    "rail_aisle_p22", "rail_aisle_p44"
  };

  double best_abs_offset = std::numeric_limits<double>::infinity();
  int best_idx = -1;
  for (size_t i = 0; i < AISLE_CENTERS.size(); ++i) {
    const double offset = y - AISLE_CENTERS[i];
    if (std::abs(offset) < best_abs_offset) {
      best_abs_offset = std::abs(offset);
      best_idx = static_cast<int>(i);
    }
  }

  if (best_idx >= 0 && best_abs_offset <= aisle_half_width) {
    r.zone = AISLE_NAMES[best_idx];
    r.aisle_y_center = AISLE_CENTERS[best_idx];
    r.rail_offset_lat = y - r.aisle_y_center;
    // Rail axis is +X in map frame (rails are parallel to X).
    // Yaw error = angle between robot heading and +X.
    r.rail_yaw_error = wrap_to_pi(yaw);
    // Confidence tapers linearly from 1.0 at center to 0.2 at half_width edge.
    r.confidence = 1.0 - 0.8 * (best_abs_offset / aisle_half_width);
  } else {
    // Inside a rail section but BETWEEN aisles — on top of a crop row.
    // This is an error state the robot should never occupy while moving.
    r.zone = "unknown";
    r.confidence = 0.0;
  }

  return r;
}

}  // namespace agv_zone_detector
