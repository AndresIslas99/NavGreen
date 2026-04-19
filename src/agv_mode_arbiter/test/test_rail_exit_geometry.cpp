#include <gtest/gtest.h>

#include "agv_mode_arbiter/rail_exit_geometry.hpp"

using agv_mode_arbiter::compute_rail_exit;
using agv_mode_arbiter::RailExitGeometryParams;

namespace {

RailExitGeometryParams greenhouse_params() {
  RailExitGeometryParams p;
  p.tag_x_rear  = 4.0;
  p.tag_x_front = 7.0;
  p.gap_x_min   = 3.5;
  p.gap_x_max   = 7.5;
  p.push_m      = 1.5;
  return p;
}

}  // namespace

TEST(RailExitGeometry, InsideRearRailPushesOutwardToPastTag) {
  // Robot deep inside REAR rail. Clearance is negative (haven't reached tag
  // yet); push target sits 1.5 m past the REAR tag toward the gap.
  const auto g = compute_rail_exit(1.0, greenhouse_params());
  EXPECT_FALSE(g.skip_push);
  EXPECT_NEAR(g.clearance_m, 1.0 - 4.0, 1e-9);      // -3.0 m
  EXPECT_NEAR(g.push_goal_x, 4.0 + 1.5, 1e-9);      // 5.5
}

TEST(RailExitGeometry, InsideFrontRailPushesOutwardToPastTag) {
  // Robot deep inside FRONT rail. Outward is -x; push target is 1.5 m
  // past the FRONT tag toward the gap.
  const auto g = compute_rail_exit(10.0, greenhouse_params());
  EXPECT_FALSE(g.skip_push);
  EXPECT_NEAR(g.clearance_m, -(10.0 - 7.0), 1e-9);  // -3.0 m (negative = not past tag)
  EXPECT_NEAR(g.push_goal_x, 7.0 - 1.5, 1e-9);      // 5.5
}

TEST(RailExitGeometry, AlreadyPastRearTagSkipsPush) {
  // Robot at 5.5 m (already ≥ 1 m past the REAR tag at 4.0). Arbiter
  // should skip the push; FSM's own release gate will fire next tick.
  const auto g = compute_rail_exit(5.5, greenhouse_params());
  EXPECT_TRUE(g.skip_push);
  EXPECT_NEAR(g.clearance_m, 1.5, 1e-9);
  EXPECT_TRUE(std::isnan(g.push_goal_x));
}

TEST(RailExitGeometry, AlreadyPastFrontTagSkipsPush) {
  // Robot at 5.5 m from the FRONT side — equivalently 1.5 m past the
  // FRONT tag (7.0) outward. Skip push, FSM releases.
  // Note: 5.5 is in the gap; the helper picks the nearer tag side.
  // Closer to 7.0 side when between tags? 5.5 - 4.0 = 1.5; 7.0 - 5.5 = 1.5.
  // Tie goes to REAR per the implementation's `<` check, so use 5.6 to
  // make it unambiguous FRONT-nearest.
  const auto g = compute_rail_exit(5.6, greenhouse_params());
  EXPECT_TRUE(g.skip_push);
  // Nearest tag is FRONT (7.0); outward is -1; clearance = -(5.6-7.0) = 1.4 m.
  EXPECT_NEAR(g.clearance_m, 1.4, 1e-9);
  EXPECT_TRUE(std::isnan(g.push_goal_x));
}
