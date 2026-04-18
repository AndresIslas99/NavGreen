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
  EXPECT_EQ(r.approach_tag_id, -1);  // In rail, not in approach strip.
}

// ── RAIL_APPROACH zone tests (P2.S2) ─────────────────────────────────

TEST(ZoneClassify, ApproachRearAtAisle3) {
  // Robot at x=4.2 (inside [4.0, 4.5) REAR approach strip) and y=0 (aisle 3).
  auto r = classify(4.2, 0.0, 0.0);
  EXPECT_EQ(r.section, "APPROACH_REAR");
  EXPECT_EQ(r.zone, "rail_approach_rear");
  EXPECT_EQ(r.approach_tag_id, 35);  // REAR aisle 3 floor tag.
  EXPECT_DOUBLE_EQ(r.aisle_y_center, 0.0);
}

TEST(ZoneClassify, ApproachFrontAtAislePlus22) {
  // Robot at x=6.8 (inside (6.5, 7.0] FRONT approach strip) and y=+2.2 (aisle 4).
  auto r = classify(6.8, 2.2, 0.0);
  EXPECT_EQ(r.section, "APPROACH_FRONT");
  EXPECT_EQ(r.zone, "rail_approach_front");
  EXPECT_EQ(r.approach_tag_id, 12);  // FRONT aisle 4 floor tag.
  EXPECT_DOUBLE_EQ(r.aisle_y_center, 2.2);
}

TEST(ZoneClassify, ApproachZoneRequiresAisleAlignment) {
  // Inside approach x-range but y too far from any aisle → plain GAP.
  auto r = classify(4.2, 1.0, 0.0);  // y=1.0 is 1.0 m from aisle 0 and 1.2 m from aisle +2.2
  EXPECT_EQ(r.zone, "gap");
  EXPECT_EQ(r.section, "GAP");
  EXPECT_EQ(r.approach_tag_id, -1);
}

TEST(ZoneClassify, GapCenterStaysGap) {
  // x=5.5 is GAP but NOT in any approach strip.
  auto r = classify(5.5, 0.0, 0.0);
  EXPECT_EQ(r.zone, "gap");
  EXPECT_EQ(r.section, "GAP");
  EXPECT_EQ(r.approach_tag_id, -1);
}

TEST(ZoneClassify, ApproachFiveAislesAllHaveTags) {
  // Each REAR aisle resolves to its unique floor tag ID.
  const std::array<std::pair<double, int>, 5> cases = {{
    {-4.4, 33}, {-2.2, 34}, {0.0, 35}, {2.2, 36}, {4.4, 37},
  }};
  for (const auto &[y, expected_id] : cases) {
    auto r = classify(4.25, y, 0.0);
    EXPECT_EQ(r.zone, "rail_approach_rear") << " y=" << y;
    EXPECT_EQ(r.approach_tag_id, expected_id) << " y=" << y;
  }
}

TEST(ZoneClassify, ApproachStripsDoNotLeakIntoRail) {
  // x=4.5 exactly — just past REAR approach upper bound; should be GAP,
  // not APPROACH. Prevents double-labelling at the strip boundary.
  auto r_edge = classify(4.5, 0.0, 0.0);
  EXPECT_EQ(r_edge.section, "GAP");
  // x=3.5 is REAR end (closed interval) → REAR, not APPROACH.
  auto r_rear = classify(3.5, 0.0, 0.0);
  EXPECT_EQ(r_rear.section, "REAR");
}
