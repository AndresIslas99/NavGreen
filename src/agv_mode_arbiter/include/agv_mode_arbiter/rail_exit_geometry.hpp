// ROS-free helper for the RAIL_EXIT push goal + clearance computation.
//
// The arbiter's Stage-M flow needs to know (a) how far past the exit
// AprilTag the robot has travelled and (b) where to push the next
// rail_driver goal so the robot clears the rail by ≥ 1 m along the
// aisle's outward axis. Both depend purely on the robot's map-frame
// `current_x` and the greenhouse's two rail sections separated by a
// 4 m gap — no cached entry pose, no travel-history heuristic.
//
// Greenhouse topology (params, not constants):
//   rear  rails: x ≤ gap_x_min  → approach tag at tag_x_rear,   outward = +1
//   front rails: x ≥ gap_x_max  → approach tag at tag_x_front,  outward = -1
//   gap   region: gap_x_min < x < gap_x_max (no push needed, already clear)

#pragma once

#include <cmath>

namespace agv_mode_arbiter {

struct RailExitGeometryParams {
  double tag_x_rear;   // REAR approach floor-tag X (map frame)
  double tag_x_front;  // FRONT approach floor-tag X
  double gap_x_min;    // End of REAR rail section (start of gap)
  double gap_x_max;    // Start of FRONT rail section
  double push_m;       // Distance past the exit tag the push-goal targets
};

struct RailExitGeometry {
  // Signed distance past the exit tag along the outward axis. Positive
  // means the robot is past the tag. NaN means the robot is outside
  // both sections (ambiguous).
  double clearance_m = 0.0;
  // Map-frame X for the push goal (NaN if skip_push).
  double push_goal_x = std::nan("");
  // True when the robot is already ≥ 1 m past the tag; arbiter should
  // not publish a new goal (the FSM release gate will fire on next tick).
  bool skip_push = false;
};

inline RailExitGeometry compute_rail_exit(double current_x,
                                          const RailExitGeometryParams &p) {
  RailExitGeometry out;
  double tag_x = 0.0;
  double outward = 0.0;
  if (current_x <= p.gap_x_min) {
    tag_x = p.tag_x_rear;
    outward = +1.0;
  } else if (current_x >= p.gap_x_max) {
    tag_x = p.tag_x_front;
    outward = -1.0;
  } else {
    // Already in gap — pick whichever tag is closer so the clearance
    // metric still makes sense (distance past the nearer tag, outward
    // from it). This is the case where the robot entered RAIL_EXIT with
    // a very short rail stretch; no push needed.
    if (std::abs(current_x - p.tag_x_rear) <
        std::abs(current_x - p.tag_x_front)) {
      tag_x = p.tag_x_rear;
      outward = +1.0;
    } else {
      tag_x = p.tag_x_front;
      outward = -1.0;
    }
    out.skip_push = true;
  }
  out.clearance_m = outward * (current_x - tag_x);
  if (out.clearance_m >= 1.0) {
    out.skip_push = true;
  }
  if (!out.skip_push) {
    out.push_goal_x = tag_x + outward * p.push_m;
  }
  return out;
}

}  // namespace agv_mode_arbiter
