#include <gtest/gtest.h>

#include "agv_mode_arbiter/mode_fsm.hpp"

using agv_mode_arbiter::FsmInputs;
using agv_mode_arbiter::Mode;
using agv_mode_arbiter::Source;
using agv_mode_arbiter::mode_to_str;
using agv_mode_arbiter::source_to_str;
using agv_mode_arbiter::step;

namespace {

FsmInputs base_inputs() {
  FsmInputs in;
  in.operator_mode = "nav";
  in.zone = "gap";
  in.rail_approach_state = "idle";
  in.rail_driver_state = "idle";
  in.safety_stop = false;
  in.approach_request_in_flight = false;
  // Default true for the existing transition tests — most of them assert
  // the auto-trigger path. Tests that specifically validate the off path
  // set auto_approach=false explicitly.
  in.auto_approach = true;
  return in;
}

}  // namespace

TEST(ModeFsm, CorridorStaysCorridorInGap) {
  auto out = step(Mode::CORRIDOR_NAV, base_inputs());
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_EQ(out.active_source, Source::NAV);
  EXPECT_FALSE(out.request_rail_approach);
}

TEST(ModeFsm, CorridorEntersApproachOnZoneAlignment) {
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  auto out = step(Mode::CORRIDOR_NAV, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_PEND);
  // Keep source=NAV so Nav2 can drive the coarse approach that
  // rail_approach has delegated to it. Zeroing would freeze the robot.
  EXPECT_EQ(out.active_source, Source::NAV);
  EXPECT_TRUE(out.request_rail_approach);
}

TEST(ModeFsm, CorridorHoldsPendingWhileServiceInFlight) {
  auto in = base_inputs();
  in.zone = "rail_approach_front";
  in.approach_request_in_flight = true;  // arbiter already dispatched call
  auto out = step(Mode::CORRIDOR_NAV, in);
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_FALSE(out.request_rail_approach);
}

TEST(ModeFsm, PendToActiveOnDrivingReport) {
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "driving";
  auto out = step(Mode::RAIL_APPROACH_PEND, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_ACTIVE);
  EXPECT_EQ(out.active_source, Source::APPROACH);
}

TEST(ModeFsm, PendAbortsFallsBackToCorridor) {
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "aborted";
  auto out = step(Mode::RAIL_APPROACH_PEND, in);
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_EQ(out.active_source, Source::NAV);
}

TEST(ModeFsm, ActiveToRailDriveOnSettled) {
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "settled";
  auto out = step(Mode::RAIL_APPROACH_ACTIVE, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_DRIVE);
  EXPECT_EQ(out.active_source, Source::RAIL);
  EXPECT_TRUE(out.request_rail_drive_goal);
}

TEST(ModeFsm, ActiveStaysActiveWhileDriving) {
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "driving";
  auto out = step(Mode::RAIL_APPROACH_ACTIVE, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_ACTIVE);
  EXPECT_EQ(out.active_source, Source::APPROACH);
}

TEST(ModeFsm, RailDriveReachedEntersRailExitWithPushRequest) {
  // Stage M: reaching rail_driver's goal does NOT release to Nav2.
  // Arbiter must enter RAIL_EXIT with source=RAIL and ask for an extended
  // goal 1 m past the exit tag so the robot drives out with wz=0.
  auto in = base_inputs();
  in.zone = "gap";
  in.rail_driver_state = "reached";
  auto out = step(Mode::RAIL_DRIVE, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_EXIT);
  EXPECT_EQ(out.active_source, Source::RAIL);
  EXPECT_TRUE(out.request_rail_exit_push);
}

TEST(ModeFsm, RailDriveAbortInsideRailStaysWithRail) {
  // blocked_lateral / blocked_misaligned inside a rail aisle must NOT hand
  // back to Nav2 — Nav2 could rotate into crop rows. Stay on RAIL so the
  // operator can issue a reverse goal to back out.
  auto in = base_inputs();
  in.zone = "rail_aisle_p22";
  in.rail_driver_state = "blocked_lateral";
  auto out = step(Mode::RAIL_DRIVE, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_EXIT);
  EXPECT_EQ(out.active_source, Source::RAIL);
  EXPECT_FALSE(out.request_rail_exit_push);
}

TEST(ModeFsm, RailExitHoldsWhileInsideRailZone) {
  // Even if rail_driver reports "reached", zone still "rail_aisle_*" means
  // the robot has not physically cleared the aisle. Stay in RAIL_EXIT.
  auto in = base_inputs();
  in.zone = "rail_aisle_0";
  in.rail_driver_state = "reached";
  in.rail_exit_clearance_m = 1.5;  // would satisfy clearance on its own
  auto out = step(Mode::RAIL_EXIT, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_EXIT);
  EXPECT_EQ(out.active_source, Source::RAIL);
}

TEST(ModeFsm, RailExitHoldsUntilClearanceMet) {
  // Zone is corridor but we're still within 1 m of the exit tag. The 1 m
  // no-rotation guard must keep us on RAIL.
  auto in = base_inputs();
  in.zone = "gap";
  in.rail_driver_state = "reached";
  in.rail_exit_clearance_m = 0.4;  // < 1.0
  auto out = step(Mode::RAIL_EXIT, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_EXIT);
  EXPECT_EQ(out.active_source, Source::RAIL);
}

TEST(ModeFsm, RailExitReleasesToCorridorOnFullClearance) {
  // All three conditions met: rail_driver reached, zone not rail/approach,
  // clearance ≥ 1 m. Hand over to Nav2.
  auto in = base_inputs();
  in.zone = "gap";
  in.rail_driver_state = "reached";
  in.rail_exit_clearance_m = 1.25;
  auto out = step(Mode::RAIL_EXIT, in);
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_EQ(out.active_source, Source::NAV);
}

TEST(ModeFsm, SafetyStopOverridesEverything) {
  for (Mode m : {Mode::CORRIDOR_NAV, Mode::RAIL_APPROACH_PEND,
                 Mode::RAIL_APPROACH_ACTIVE, Mode::RAIL_DRIVE,
                 Mode::RAIL_EXIT}) {
    auto in = base_inputs();
    in.safety_stop = true;
    in.zone = "rail_aisle_0";
    in.rail_driver_state = "driving";
    auto out = step(m, in);
    EXPECT_EQ(out.next_mode, Mode::BLOCKED_HANDOFF)
        << "from mode " << mode_to_str(m);
    EXPECT_EQ(out.active_source, Source::NONE);
  }
}

TEST(ModeFsm, BlockedRecoversToCorridor) {
  auto in = base_inputs();
  in.safety_stop = false;  // stop cleared
  auto out = step(Mode::BLOCKED_HANDOFF, in);
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_EQ(out.active_source, Source::NAV);
}

TEST(ModeFsm, OperatorIdleOverridesFromAnyMode) {
  for (Mode m : {Mode::CORRIDOR_NAV, Mode::RAIL_APPROACH_ACTIVE,
                 Mode::RAIL_DRIVE}) {
    auto in = base_inputs();
    in.operator_mode = "idle";
    in.zone = "rail_aisle_0";
    auto out = step(m, in);
    EXPECT_EQ(out.next_mode, Mode::IDLE);
    EXPECT_EQ(out.active_source, Source::NONE);
  }
}

TEST(ModeFsm, OperatorTeleopOverrides) {
  auto in = base_inputs();
  in.operator_mode = "teleop";
  auto out = step(Mode::RAIL_DRIVE, in);
  EXPECT_EQ(out.next_mode, Mode::TELEOP);
  EXPECT_EQ(out.active_source, Source::NONE);
}

TEST(ModeFsm, ApproachOnFrontAisleAlsoTriggersEntry) {
  auto in = base_inputs();
  in.zone = "rail_approach_front";
  auto out = step(Mode::CORRIDOR_NAV, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_PEND);
  EXPECT_TRUE(out.request_rail_approach);
}

TEST(ModeFsm, DirectRailDriverGoalShortcutsToRailDrive) {
  // Stage G+I fix: the dispatch router publishes directly to
  // /agv/rail_driver/goal and rail_driver transitions to state=="driving".
  // The arbiter must pick that up from CORRIDOR_NAV and relay cmd_vel_rail.
  auto in = base_inputs();
  in.rail_driver_state = "driving";
  auto out = step(Mode::CORRIDOR_NAV, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_DRIVE);
  EXPECT_EQ(out.active_source, Source::RAIL);
}

TEST(ModeFsm, AutoApproachOffStaysInCorridor) {
  auto in = base_inputs();
  in.auto_approach = false;
  in.zone = "rail_approach_rear";
  auto out = step(Mode::CORRIDOR_NAV, in);
  // Without auto_approach, the arbiter stays CORRIDOR_NAV so Nav2 keeps
  // driving straight through the approach strip. rail_approach must be
  // fired externally to engage.
  EXPECT_EQ(out.next_mode, Mode::CORRIDOR_NAV);
  EXPECT_EQ(out.active_source, Source::NAV);
  EXPECT_FALSE(out.request_rail_approach);
}

TEST(ModeFsm, AutoApproachOffStillTracksExternalApproach) {
  // Auto-trigger off, but rail_approach was fired externally and reached
  // fine_servoing. The arbiter must still hand over cmd_vel to APPROACH.
  auto in = base_inputs();
  in.auto_approach = false;
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "driving";
  auto out = step(Mode::CORRIDOR_NAV, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_ACTIVE);
  EXPECT_EQ(out.active_source, Source::APPROACH);
}

TEST(ModeFsm, PendKeepsNavWhileRailApproachCoarsing) {
  // While rail_approach runs its COARSE_APPROACH sub-phase via Nav2, the
  // arbiter must relay Nav2 (not hold 0) so the coarse approach finishes.
  auto in = base_inputs();
  in.zone = "rail_approach_rear";
  in.rail_approach_state = "coarse_approach";  // rail_approach in Nav2-driven phase
  auto out = step(Mode::RAIL_APPROACH_PEND, in);
  EXPECT_EQ(out.next_mode, Mode::RAIL_APPROACH_PEND);
  EXPECT_EQ(out.active_source, Source::NAV);
}

TEST(ModeFsm, LabelsCoverAllStates) {
  EXPECT_STREQ(mode_to_str(Mode::CORRIDOR_NAV), "corridor_nav");
  EXPECT_STREQ(mode_to_str(Mode::RAIL_APPROACH_PEND), "rail_approach_pend");
  EXPECT_STREQ(mode_to_str(Mode::RAIL_APPROACH_ACTIVE), "rail_approach_active");
  EXPECT_STREQ(mode_to_str(Mode::RAIL_DRIVE), "rail_drive");
  EXPECT_STREQ(mode_to_str(Mode::RAIL_EXIT), "rail_exit");
  EXPECT_STREQ(mode_to_str(Mode::BLOCKED_HANDOFF), "blocked_handoff");
  EXPECT_STREQ(mode_to_str(Mode::TELEOP), "teleop");
  EXPECT_STREQ(mode_to_str(Mode::IDLE), "idle");

  EXPECT_STREQ(source_to_str(Source::NONE), "none");
  EXPECT_STREQ(source_to_str(Source::NAV), "nav");
  EXPECT_STREQ(source_to_str(Source::APPROACH), "approach");
  EXPECT_STREQ(source_to_str(Source::RAIL), "rail");
}
