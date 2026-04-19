#include <cmath>
#include <gtest/gtest.h>

#include "agv_rail_driver/rail_controller.hpp"

using agv_rail_driver::RailControllerInputs;
using agv_rail_driver::RailControllerParams;
using agv_rail_driver::RailState;
using agv_rail_driver::compute;
using agv_rail_driver::state_to_str;

namespace {

RailControllerInputs base_inputs() {
  RailControllerInputs in;
  in.current_x = 0.0;
  in.current_y = 0.0;
  in.goal_x = 1.0;
  in.goal_y = 0.0;
  in.rail_axis_sign = 1.0;
  in.rail_yaw_error = 0.0;
  in.in_rail_zone = false;
  in.collision_monitor_stop = false;
  in.have_goal = true;
  return in;
}

}  // namespace

TEST(RailController, IdleNoGoal) {
  RailControllerInputs in = base_inputs();
  in.have_goal = false;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::IDLE);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

TEST(RailController, NominalDrivingCommandsForward) {
  auto in = base_inputs();
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
  EXPECT_GT(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

TEST(RailController, AngularAlwaysZero) {
  // Exhaustive: regardless of any input combination, angular_z must stay 0.
  for (double yaw_err : {-0.5, -0.1, 0.0, 0.1, 0.5}) {
    auto in = base_inputs();
    in.in_rail_zone = true;
    in.rail_yaw_error = yaw_err;
    auto out = compute(in, {});
    EXPECT_DOUBLE_EQ(out.angular_z, 0.0) << "yaw_err=" << yaw_err;
  }
}

TEST(RailController, ReachedWithinStopBand) {
  auto in = base_inputs();
  in.current_x = 0.98;  // 2 cm from goal 1.0
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::REACHED);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
}

TEST(RailController, CollisionMonitorHoldsZeroIndefinitely) {
  auto in = base_inputs();
  in.collision_monitor_stop = true;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_WAIT);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
}

TEST(RailController, LateralDriftAborts) {
  auto in = base_inputs();
  in.current_y = 0.5;  // default lateral_abort_m = 0.30
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_LATERAL);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
}

TEST(RailController, YawAbortOnlyInsideRailZone) {
  auto in = base_inputs();
  in.rail_yaw_error = 0.4;  // > 0.26 rad default
  // Without rail zone flag, yaw abort does NOT trigger.
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);

  in.in_rail_zone = true;
  out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_MISALIGNED);
}

TEST(RailController, SpeedCap) {
  RailControllerParams p;
  p.kP = 10.0;
  p.speed_max_mps = 0.5;
  auto in = base_inputs();
  in.current_x = 0.0;
  in.goal_x = 100.0;
  auto out = compute(in, p);
  EXPECT_NEAR(out.linear_x, 0.5, 1e-9);
}

TEST(RailController, ReverseWhenRobotMisaligned) {
  // Robot at (0,0) facing +X (yaw=0), goal at (-1, 0). Body-frame err = -1,
  // so the controller commands negative linear.x — which, with wz=0, the
  // robot cannot act on. Upstream is responsible for aligning the robot to
  // face -X before handing off.
  auto in = base_inputs();
  in.current_x = 0.0;
  in.goal_x = -1.0;
  in.current_yaw = 0.0;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
  EXPECT_LT(out.linear_x, 0.0);  // controller says reverse; wz=0
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

TEST(RailController, YawPiForwardDrivesTowardLowerWorldX) {
  // Robot at (4, 0) facing world -X (yaw=π), goal at (1, 0). Body +X equals
  // world -X at this yaw, so the controller must command POSITIVE linear.x
  // to drive the robot toward world (1, 0). Regression test for Round 42b
  // wp05 where the old sign-based controller drove the robot backward.
  auto in = base_inputs();
  in.current_x = 4.0;
  in.goal_x = 1.0;
  in.goal_y = 0.0;
  in.current_yaw = M_PI;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
  EXPECT_GT(out.linear_x, 0.0) << "with yaw=π, forward motion reduces world_x";
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
}

TEST(RailController, RemainingDistanceShrinksAsRobotApproaches) {
  auto in = base_inputs();
  auto out1 = compute(in, {});
  in.current_x = 0.5;
  auto out2 = compute(in, {});
  EXPECT_GT(out1.remaining_m, out2.remaining_m);
}

TEST(RailController, CollisionMonitorTakesPriorityOverLateralAbort) {
  // If both triggers fire simultaneously, collision_monitor wins (safety).
  auto in = base_inputs();
  in.collision_monitor_stop = true;
  in.current_y = 0.5;  // would have been BLOCKED_LATERAL
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_WAIT);
}

TEST(RailController, YawNanIsTolerated) {
  // Gap-zone operation: zone_detector returns NaN for rail_yaw_error.
  auto in = base_inputs();
  in.in_rail_zone = true;
  in.rail_yaw_error = std::nan("");
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
}

TEST(RailController, StateToStringCoverage) {
  // Sanity: every enum value has a readable label.
  EXPECT_STREQ(state_to_str(RailState::IDLE), "idle");
  EXPECT_STREQ(state_to_str(RailState::DRIVING), "driving");
  EXPECT_STREQ(state_to_str(RailState::REACHED), "reached");
  EXPECT_STREQ(state_to_str(RailState::BLOCKED_WAIT), "blocked_wait");
  EXPECT_STREQ(state_to_str(RailState::BLOCKED_MISALIGNED), "blocked_misaligned");
  EXPECT_STREQ(state_to_str(RailState::BLOCKED_LATERAL), "blocked_lateral");
}

// ── Stage K: visual rail feedback preferred when fresh+confident ──────────

TEST(RailController, VisualPreferredOverPose) {
  // Pose says we're on the goal centerline (no drift). Visual says otherwise:
  // the rail midline is 40 cm off in base_link Y, above lateral_abort_m 0.30.
  // Visual must win → BLOCKED_LATERAL.
  auto in = base_inputs();
  in.current_y = 0.0;
  in.goal_y = 0.0;
  in.visual_confidence = 0.9;
  in.visual_age_s = 0.1;
  in.visual_lat_offset = 0.40;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_LATERAL);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
}

TEST(RailController, VisualStaleFallsBackToPose) {
  // Visual says huge drift, but age=2s is older than visual_max_age_s (0.5 s
  // default). Controller must ignore the stale visual and trust the pose
  // (which shows no drift) → DRIVING, not BLOCKED_LATERAL.
  auto in = base_inputs();
  in.current_y = 0.0;
  in.goal_y = 0.0;
  in.visual_confidence = 0.95;
  in.visual_age_s = 2.0;
  in.visual_lat_offset = 0.40;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
}

TEST(RailController, VisualLowConfidenceRejected) {
  // Visual is fresh but confidence 0.3 is below the 0.7 threshold. Must
  // fall back to pose metrics.
  auto in = base_inputs();
  in.current_y = 0.0;
  in.goal_y = 0.0;
  in.visual_confidence = 0.3;
  in.visual_age_s = 0.1;
  in.visual_lat_offset = 0.40;
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::DRIVING);
}

TEST(RailController, VisualMissingUsesPose) {
  // Default: visual never received (age=inf, conf=0). Pose-based check
  // triggers on current_y drift.
  auto in = base_inputs();
  in.current_y = 0.5;  // > lateral_abort_m 0.30
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_LATERAL);
}

TEST(RailController, VisualYawAbortInsideRailZone) {
  // Visual says yaw is 30° off the rail axis (> yaw_abort_rad 0.26 rad). In a
  // rail aisle with visual trusted, BLOCKED_MISALIGNED must fire from the
  // visual input even when zone_detector's rail_yaw_error is clean.
  auto in = base_inputs();
  in.in_rail_zone = true;
  in.rail_yaw_error = 0.0;      // pose says we're aligned
  in.visual_confidence = 0.9;
  in.visual_age_s = 0.1;
  in.visual_yaw_error = 0.52;   // ~30°
  auto out = compute(in, {});
  EXPECT_EQ(out.state, RailState::BLOCKED_MISALIGNED);
}
