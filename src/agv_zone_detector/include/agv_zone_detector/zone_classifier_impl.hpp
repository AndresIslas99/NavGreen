#pragma once

#include "agv_zone_detector/zone_classifier.hpp"

namespace agv_zone_detector {

inline ClassifyResult classify(double x, double y, double yaw,
                               double aisle_half_width) {
  ClassifyResult r;
  r.aisle_y_center = std::nan("");
  r.rail_offset_lat = std::nan("");
  r.rail_yaw_error = std::nan("");
  r.approach_tag_id = -1;

  // Section lookup.
  //   REAR           x ∈ [-16.5,  3.5]
  //   APPROACH_REAR  x ∈ [ 4.0,   4.5) — 0.5 m strip east of REAR end
  //   GAP            x ∈ ( 3.5,   7.5)
  //   APPROACH_FRONT x ∈ ( 6.5,   7.0] — 0.5 m strip west of FRONT start
  //   FRONT          x ∈ [ 7.5,  27.5]
  //
  // The approach strips are sub-zones of GAP so GAP spans the full 4 m;
  // the classifier prefers the approach label when the aisle aligns.
  constexpr double REAR_X_START  = -16.5;
  constexpr double REAR_X_END    =   3.5;
  constexpr double FRONT_X_START =   7.5;
  constexpr double FRONT_X_END   =  27.5;
  constexpr double APPROACH_REAR_X_LO  = 4.0;
  constexpr double APPROACH_REAR_X_HI  = 4.5;
  constexpr double APPROACH_FRONT_X_LO = 6.5;
  constexpr double APPROACH_FRONT_X_HI = 7.0;

  // Aisle lookup (shared by rail and approach branches).
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
  const bool aisle_in_range =
      (best_idx >= 0 && best_abs_offset <= aisle_half_width);

  const bool in_approach_rear =
      (x >= APPROACH_REAR_X_LO  && x <  APPROACH_REAR_X_HI);
  const bool in_approach_front =
      (x >  APPROACH_FRONT_X_LO && x <= APPROACH_FRONT_X_HI);

  // Approach strips — only label if the robot is also aisle-aligned.
  // Otherwise fall through to plain GAP.
  if (in_approach_rear && aisle_in_range) {
    r.section = "APPROACH_REAR";
    r.zone = "rail_approach_rear";
    r.aisle_y_center = AISLE_CENTERS[best_idx];
    r.rail_offset_lat = y - r.aisle_y_center;
    // Rails run along the +X/-X axis; either direction is a valid
    // traversal. Pick the smaller-magnitude signed error vs {0, π} so
    // a robot facing -X (yaw≈π) reports rail_yaw_error ≈ 0 — otherwise
    // rail_driver's BLOCKED_MISALIGNED kicks in on correct reverse-
    // direction goals. wp05 of Round 42c was the repro case.
    {
      const double err_fwd = wrap_to_pi(yaw);
      const double err_rev = wrap_to_pi(yaw - M_PI);
      r.rail_yaw_error = (std::abs(err_fwd) <= std::abs(err_rev))
          ? err_fwd : err_rev;
    }
    r.approach_tag_id = tag_id_for_rear_approach(best_idx);
    r.confidence = 1.0 - 0.8 * (best_abs_offset / aisle_half_width);
    return r;
  }
  if (in_approach_front && aisle_in_range) {
    r.section = "APPROACH_FRONT";
    r.zone = "rail_approach_front";
    r.aisle_y_center = AISLE_CENTERS[best_idx];
    r.rail_offset_lat = y - r.aisle_y_center;
    // Rails run along the +X/-X axis; either direction is a valid
    // traversal. Pick the smaller-magnitude signed error vs {0, π} so
    // a robot facing -X (yaw≈π) reports rail_yaw_error ≈ 0 — otherwise
    // rail_driver's BLOCKED_MISALIGNED kicks in on correct reverse-
    // direction goals. wp05 of Round 42c was the repro case.
    {
      const double err_fwd = wrap_to_pi(yaw);
      const double err_rev = wrap_to_pi(yaw - M_PI);
      r.rail_yaw_error = (std::abs(err_fwd) <= std::abs(err_rev))
          ? err_fwd : err_rev;
    }
    r.approach_tag_id = tag_id_for_front_approach(best_idx);
    r.confidence = 1.0 - 0.8 * (best_abs_offset / aisle_half_width);
    return r;
  }

  const bool in_rear  = (x >= REAR_X_START  && x <= REAR_X_END);
  const bool in_front = (x >= FRONT_X_START && x <= FRONT_X_END);
  const bool in_gap   = (x >  REAR_X_END    && x <  FRONT_X_START);

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

  // Inside a rail section — determine which aisle (if any).
  if (aisle_in_range) {
    r.zone = AISLE_NAMES[best_idx];
    r.aisle_y_center = AISLE_CENTERS[best_idx];
    r.rail_offset_lat = y - r.aisle_y_center;
    // Rails run along the +X/-X axis; either direction is a valid
    // traversal. Pick the smaller-magnitude signed error vs {0, π} so
    // a robot facing -X (yaw≈π) reports rail_yaw_error ≈ 0 — otherwise
    // rail_driver's BLOCKED_MISALIGNED kicks in on correct reverse-
    // direction goals. wp05 of Round 42c was the repro case.
    {
      const double err_fwd = wrap_to_pi(yaw);
      const double err_rev = wrap_to_pi(yaw - M_PI);
      r.rail_yaw_error = (std::abs(err_fwd) <= std::abs(err_rev))
          ? err_fwd : err_rev;
    }
    r.confidence = 1.0 - 0.8 * (best_abs_offset / aisle_half_width);
  } else {
    r.zone = "unknown";
    r.confidence = 0.0;
  }

  return r;
}

}  // namespace agv_zone_detector
