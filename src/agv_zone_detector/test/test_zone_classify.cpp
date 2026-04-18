#include <cmath>
#include <gtest/gtest.h>

#include "agv_zone_detector/zone_classifier.hpp"
#include "agv_zone_detector/zone_classifier_impl.hpp"

using agv_zone_detector::classify;

TEST(ZoneClassify, GapCenterIsGap) {
  auto r = classify(5.5, 0.0, 0.0);
  EXPECT_EQ(r.section, "GAP");
  EXPECT_EQ(r.zone, "gap");
  EXPECT_DOUBLE_EQ(r.confidence, 1.0);
}

TEST(ZoneClassify, GapBoundariesAreGap) {
  auto r1 = classify(3.6, 0.0, 0.0);    // just east of REAR end
  EXPECT_EQ(r1.section, "GAP");
  auto r2 = classify(7.4, 0.0, 0.0);    // just west of FRONT start
  EXPECT_EQ(r2.section, "GAP");
}

TEST(ZoneClassify, RearSectionAisleCenter) {
  auto r = classify(0.0, 0.0, 0.0);     // round 37 case — was "west open" but really REAR aisle 3
  EXPECT_EQ(r.section, "REAR");
  EXPECT_EQ(r.zone, "rail_aisle_0");
  EXPECT_DOUBLE_EQ(r.aisle_y_center, 0.0);
}

TEST(ZoneClassify, RearSectionAislePlus22) {
  auto r = classify(0.9, 2.1, 0.0);
  EXPECT_EQ(r.section, "REAR");
  EXPECT_EQ(r.zone, "rail_aisle_p22");
  EXPECT_DOUBLE_EQ(r.aisle_y_center, 2.2);
  EXPECT_NEAR(r.rail_offset_lat, -0.1, 1e-6);
}

TEST(ZoneClassify, RearSectionAisleMinus22) {
  auto r = classify(-5.0, -2.2, 0.0);
  EXPECT_EQ(r.section, "REAR");
  EXPECT_EQ(r.zone, "rail_aisle_m22");
}

TEST(ZoneClassify, FrontSectionAisleCenter) {
  auto r = classify(15.0, 0.0, 0.0);
  EXPECT_EQ(r.section, "FRONT");
  EXPECT_EQ(r.zone, "rail_aisle_0");
  EXPECT_DOUBLE_EQ(r.aisle_y_center, 0.0);
}

TEST(ZoneClassify, OutsideWest) {
  auto r = classify(-20.0, 0.0, 0.0);
  EXPECT_EQ(r.section, "OUTSIDE");
  EXPECT_EQ(r.zone, "corridor_west");
}

TEST(ZoneClassify, OutsideEast) {
  auto r = classify(30.0, 0.0, 0.0);
  EXPECT_EQ(r.section, "OUTSIDE");
  EXPECT_EQ(r.zone, "corridor_east");
}

TEST(ZoneClassify, InRearSectionBetweenAislesIsUnknown) {
  // y=-1.1 is a crop row, not an aisle. In rail section, y at crop row is
  // the "robot should never be here" zone.
  auto r = classify(-5.0, -1.1, 0.0);
  EXPECT_EQ(r.section, "REAR");
  EXPECT_EQ(r.zone, "unknown");
  EXPECT_DOUBLE_EQ(r.confidence, 0.0);
}

TEST(ZoneClassify, YawErrorInRail) {
  auto r = classify(0.0, 0.0, 0.5);     // 0.5 rad heading error
  EXPECT_EQ(r.zone, "rail_aisle_0");
  EXPECT_NEAR(r.rail_yaw_error, 0.5, 1e-6);
}

TEST(ZoneClassify, ConfidenceTapersWithLateralOffset) {
  auto r_center = classify(0.0, 0.0, 0.0);
  auto r_edge   = classify(0.0, 0.30, 0.0);   // near aisle_half_width=0.35
  EXPECT_GT(r_center.confidence, r_edge.confidence);
  EXPECT_GT(r_edge.confidence, 0.2);
}

TEST(ZoneClassify, OperatorRound37Case) {
  // The moment that broke Phase 1: robot at (0.907, -0.107, yaw=-5.1°)
  // was in REAR aisle 3 rails, not "west open" as we thought.
  const double yaw_rad = -5.1 * M_PI / 180.0;
  auto r = classify(0.907, -0.107, yaw_rad);
  EXPECT_EQ(r.section, "REAR");
  EXPECT_EQ(r.zone, "rail_aisle_0");
  EXPECT_NEAR(r.rail_offset_lat, -0.107, 1e-6);
}
