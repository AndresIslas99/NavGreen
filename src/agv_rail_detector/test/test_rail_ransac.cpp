#include <cmath>
#include <gtest/gtest.h>

#include "agv_rail_detector/rail_ransac.hpp"

using agv_rail_detector::Line2D;
using agv_rail_detector::Point2D;
using agv_rail_detector::RailDetection;
using agv_rail_detector::RansacParams;
using agv_rail_detector::detect_rails;
using agv_rail_detector::parallel_line_distance;
using agv_rail_detector::synth_rail_points;

RansacParams base_params() {
  RansacParams p;
  p.pair_spacing_m = 0.45;
  p.spacing_tolerance_m = 0.03;
  p.inlier_tolerance_m = 0.02;
  p.min_inliers = 8;
  p.max_iterations = 500;
  p.min_confidence = 0.3;
  return p;
}

TEST(RailRansac, CleanRailPairIsDetected) {
  auto pts = synth_rail_points(2.0, 0.0, 0.0, 3.0, 0.45, 20);
  auto det = detect_rails(pts, base_params());
  EXPECT_GT(det.confidence, 0.5);
  EXPECT_GE(det.inliers_left, 8);
  EXPECT_GE(det.inliers_right, 8);
  // Lines should be 0.45 m apart.
  const double dist = std::abs(parallel_line_distance(det.line_left,
                                                       det.line_right));
  EXPECT_NEAR(dist, 0.45, 0.05);
}

TEST(RailRansac, EmptyPointsReturnsZeroConfidence) {
  std::vector<Point2D> pts;
  auto det = detect_rails(pts, base_params());
  EXPECT_DOUBLE_EQ(det.confidence, 0.0);
}

TEST(RailRansac, TooFewPointsReturnsZeroConfidence) {
  std::vector<Point2D> pts = {{0, 0}, {0.1, 0}, {0.2, 0}};
  auto det = detect_rails(pts, base_params());
  EXPECT_DOUBLE_EQ(det.confidence, 0.0);
}

TEST(RailRansac, SingleRailNoParallelIsRejected) {
  // Only one rail's worth of points — RANSAC should fail to find the
  // parallel partner → confidence 0 (below min_confidence).
  std::vector<Point2D> pts;
  for (int i = 0; i < 20; ++i) {
    pts.push_back({static_cast<double>(i) * 0.1, 0.0});
  }
  auto det = detect_rails(pts, base_params());
  EXPECT_LT(det.confidence, 0.3);
}

TEST(RailRansac, OrthogonalNoiseIsRejected) {
  // 30 random points scattered in a perpendicular orientation — no
  // parallel pair exists. Confidence should be below threshold.
  std::vector<Point2D> pts;
  for (int i = 0; i < 30; ++i) {
    const double t = static_cast<double>(i) * 0.1;
    pts.push_back({t, std::sin(t * 3.0) * 0.8});
  }
  auto det = detect_rails(pts, base_params());
  EXPECT_LT(det.confidence, 0.3);
}

TEST(RailRansac, RailPairWithMinorOutliersStillDetected) {
  auto pts = synth_rail_points(2.0, 0.0, 0.0, 3.0, 0.45, 25);
  // Add 5 outliers (roughly 10 %).
  pts.push_back({1.0, 1.5});
  pts.push_back({2.3, -1.2});
  pts.push_back({0.5, 2.0});
  pts.push_back({2.8, 0.9});
  pts.push_back({2.1, -1.1});
  auto det = detect_rails(pts, base_params());
  EXPECT_GT(det.confidence, 0.4);
}

TEST(RailRansac, RotatedRailPairIsDetected) {
  // Rails at yaw=π/6 — the RANSAC should still lock onto them.
  auto pts = synth_rail_points(5.0, 2.0, M_PI / 6.0, 3.0, 0.45, 20);
  auto det = detect_rails(pts, base_params());
  EXPECT_GT(det.confidence, 0.5);
  const double dist = std::abs(parallel_line_distance(det.line_left,
                                                       det.line_right));
  EXPECT_NEAR(dist, 0.45, 0.05);
}

TEST(RailRansac, WrongSpacingIsRejected) {
  // Synth with 0.60 m spacing; params expect 0.45 m ± 0.03. No valid pair.
  auto pts = synth_rail_points(2.0, 0.0, 0.0, 3.0, 0.60, 20);
  auto det = detect_rails(pts, base_params());
  EXPECT_LT(det.confidence, 0.3);
}

TEST(RailRansac, ReverseDirectionRailDetectedSymmetric) {
  // Rails oriented along −X (yaw=π). RANSAC is direction-agnostic.
  auto pts = synth_rail_points(-3.0, 0.0, M_PI, 3.0, 0.45, 20);
  auto det = detect_rails(pts, base_params());
  EXPECT_GT(det.confidence, 0.5);
}
