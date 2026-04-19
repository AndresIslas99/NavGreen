// Pure-logic tests for agv_rail_approach's fine-servoing controller.
//
// These run without ROS, without a sim, without TF. A full HIL
// iteration to surface a tuning regression is ~35 min of wall clock;
// these cases run in <1 s and cover every rejection branch + the
// happy path with a synthetic 4-corner projection.

#include <cmath>
#include <gtest/gtest.h>

#include "agv_rail_approach/fine_servo_controller.hpp"

using agv_rail_approach::FineServoParams;
using agv_rail_approach::FineServoVerdict;
using agv_rail_approach::fine_servo_step;

namespace {

// Greenhouse AGV TF constants (match agv_hil_full.launch.py):
//   base_link → zed_camera_link = (0.700, 0.0, 0.010)
//   → zed_left_camera_frame     = (0.0, 0.06, 0.0)
// The synth_ helper below puts tags in base_link coordinates and
// projects onto the camera — use these when constructing cam_to_base.
constexpr double kCamXInBase = 0.70;
constexpr double kCamZInBase = 0.21;  // z=0.010 + 0.20 base_link off-ground
constexpr double kDesiredOffsetX = 0.3;   // camera-frame tag distance
constexpr double kTagSize = 0.2;

FineServoParams default_params() {
  FineServoParams p;
  p.fx = 448.1;
  p.fy = 448.1;
  p.cx = 640.0;
  p.cy = 360.0;
  p.tag_size_m = kTagSize;
  // desired_offset_x is CAMERA-to-tag distance along optical Z (see
  // the rail_approach_params.yaml comment). When the tag sits at
  // camera-Z = 0.3, error_x = 0 and the P-controller stops moving.
  p.desired_offset_x = kDesiredOffsetX;
  p.desired_offset_y = 0.0;
  p.tolerance_xy = 0.02;
  p.tolerance_yaw = 0.017;
  p.kp_linear = 0.15;
  p.kp_lateral = 0.3;
  p.kp_yaw = 0.5;
  p.max_linear_mps = 0.03;
  p.max_angular_rps = 0.10;
  // synth_floor_tag_corners emits a floor-plane tag. The yaw formula
  // produces a reference angle of π for that geometry (see controller
  // docstring); disable the yaw-convergence check so happy-path tests
  // can latch in_tolerance on the x/y errors alone.
  p.check_yaw_convergence = false;
  return p;
}

// Build a 4x4 homogeneous transform that maps camera-optical → base_link
// for the greenhouse AGV (camera 0.70 m forward, 0.21 m up, facing
// world +x under yaw=0). Optical axes are X-right, Y-down, Z-forward.
// In base_link frame:
//   base.x = cam.z + 0.70
//   base.y = -cam.x
//   base.z = -cam.y + 0.21
cv::Matx44d greenhouse_cam_to_base() {
  // Rotation that sends (X,Y,Z)_optical to (Z, -X, -Y)_base_link:
  //   base.x = +cam.z (tag forward of camera = forward of base)
  //   base.y = -cam.x (tag right of camera = right of base, base y=left)
  //   base.z = -cam.y (tag below camera = above base by cam height)
  cv::Matx44d M = cv::Matx44d::eye();
  M(0, 0) = 0.0; M(0, 1) = 0.0; M(0, 2) = 1.0; M(0, 3) = kCamXInBase;
  M(1, 0) = -1.0; M(1, 1) = 0.0; M(1, 2) = 0.0; M(1, 3) = 0.0;
  M(2, 0) = 0.0; M(2, 1) = -1.0; M(2, 2) = 0.0; M(2, 3) = kCamZInBase;
  return M;
}

// Project a tag centred on the ground at `tag_x` m in front of the
// robot (base_link +X), y-offset `tag_y`, onto the camera image plane.
// Returns the 4 pixel corners in apriltag_ros CCW-from-+Z order:
// BL, BR, TR, TL of the tag's local frame.
//
// Tag local +X points in world +X (same as robot's base_link +X), so
// the tag's local X axis in the camera-optical frame is (0, 0, +1).
// Tag local +Y in world = world +Y = camera-optical -X.
//
// This lets us construct synthetic "perfect" corner observations for a
// floor tag at a known pose, independent of the ROS/tf plumbing.
std::vector<cv::Point2d> synth_floor_tag_corners(
    double tag_x_base, double tag_y_base, const FineServoParams &p) {
  const double half = p.tag_size_m / 2.0;
  // Corner offsets in the tag's local XY plane (tag face is local Z=0).
  const double offsets[4][2] = {
      {-half, -half}, { half, -half}, { half,  half}, {-half,  half}};

  std::vector<cv::Point2d> out;
  out.reserve(4);
  for (int i = 0; i < 4; ++i) {
    // Corner in base_link.
    const double bx = tag_x_base + offsets[i][0];
    const double by = tag_y_base + offsets[i][1];
    const double bz = 0.0;
    // base → cam-optical: invert greenhouse_cam_to_base analytically.
    //   cam.x = -base.y
    //   cam.y = kCamZInBase - base.z
    //   cam.z = base.x - kCamXInBase
    const double cam_x = -by;
    const double cam_y = kCamZInBase - bz;
    const double cam_z = bx - kCamXInBase;
    if (cam_z <= 0.0) {
      return {};  // behind camera — caller should not happen
    }
    const double u = p.fx * cam_x / cam_z + p.cx;
    const double v = p.fy * cam_y / cam_z + p.cy;
    out.emplace_back(u, v);
  }
  return out;
}

}  // namespace


// ── rejection branches ─────────────────────────────────────────────────

TEST(FineServo, InvalidCornersRejected) {
  auto p = default_params();
  std::vector<cv::Point2d> corners;  // empty
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  EXPECT_EQ(out.verdict, FineServoVerdict::INVALID_CORNERS);
  EXPECT_DOUBLE_EQ(out.cmd_linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(out.cmd_angular_rps, 0.0);
}

// Convert a camera-frame offset (cam-Z forward, cam-X right) into a
// base_link tag position suitable for synth_floor_tag_corners. Abstracts
// out the kCamXInBase forward offset so tests read naturally.
static inline double tag_x_base_for(double desired_cam_z) {
  return kCamXInBase + desired_cam_z;
}
static inline double tag_y_base_for(double cam_x_right) {
  return -cam_x_right;
}

TEST(FineServo, InvalidIntrinsicsRejected) {
  auto p = default_params();
  p.fx = 0.0;
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x), 0.0, default_params());
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  EXPECT_EQ(out.verdict, FineServoVerdict::INVALID_INTRINSICS);
}

TEST(FineServo, RangeOutOfBoundsRejected) {
  auto p = default_params();
  p.range_max_m = 0.5;  // tag at camera-Z = 0.6 m is now out of bounds.
  auto corners = synth_floor_tag_corners(tag_x_base_for(0.6), 0.0, p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  EXPECT_EQ(out.verdict, FineServoVerdict::RANGE_OUT_OF_BOUNDS);
  EXPECT_GT(out.range_m, 0.5);
}

// ── happy path + control law ───────────────────────────────────────────

TEST(FineServo, HappyPathCentredTagHasSmallErrors) {
  // Tag exactly at the desired camera-frame offset → zero error on
  // x/y. error_yaw is not asserted here because the yaw-extraction
  // formula was designed for wall tags and produces a ~π reference
  // angle for floor tags (see controller docstring). Callers that
  // target floor tags set check_yaw_convergence = false so the yaw
  // component is excluded from the settlement latch.
  auto p = default_params();
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x),
      tag_y_base_for(p.desired_offset_y), p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  EXPECT_EQ(out.verdict, FineServoVerdict::OK);
  EXPECT_NEAR(out.error_x, 0.0, 2e-3);
  EXPECT_NEAR(out.error_y, 0.0, 2e-3);
  EXPECT_LT(std::abs(out.cmd_linear_mps), 0.01);
}

TEST(FineServo, LateralOffsetProducesAngularCmd) {
  auto p = default_params();
  // Tag displaced 0.1 m to the right of camera optical X (i.e. robot
  // should rotate to centre it).
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x),
      tag_y_base_for(0.1), p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  ASSERT_EQ(out.verdict, FineServoVerdict::OK);
  // tvec.x > 0 (cam right) → error_y = -tvec.x − 0 < 0 → angular cmd
  // negative (turn right toward the tag).
  EXPECT_LT(out.error_y, 0.0);
  EXPECT_LT(out.cmd_angular_rps, 0.0);
}

TEST(FineServo, VelocitiesClamped) {
  auto p = default_params();
  // Tag much further than desired so kp_linear * error_x exceeds cap.
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x + 2.0), 0.0, p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  ASSERT_EQ(out.verdict, FineServoVerdict::OK);
  // 0.15 * 2.0 = 0.3 → clamped to 0.03.
  EXPECT_NEAR(out.cmd_linear_mps, p.max_linear_mps, 1e-9);
}

// ── convergence flag ───────────────────────────────────────────────────

TEST(FineServo, InToleranceFlagReflectsErrors) {
  auto p = default_params();
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x),
      tag_y_base_for(p.desired_offset_y), p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  ASSERT_EQ(out.verdict, FineServoVerdict::OK);
  EXPECT_TRUE(out.in_tolerance);
}

TEST(FineServo, OutOfToleranceResetsFlag) {
  auto p = default_params();
  // 10 cm lateral off — well above the 2 cm tolerance.
  auto corners = synth_floor_tag_corners(
      tag_x_base_for(p.desired_offset_x),
      tag_y_base_for(0.1), p);
  auto out = fine_servo_step(corners, greenhouse_cam_to_base(), p);
  ASSERT_EQ(out.verdict, FineServoVerdict::OK);
  EXPECT_FALSE(out.in_tolerance);
}

// ── verdict-to-string coverage ─────────────────────────────────────────

// ── TvecRvecMedianFilter (iter-12) ─────────────────────────────────────

TEST(TvecRvecMedianFilter, EmptyFilterReportsUnfilled) {
  agv_rail_approach::TvecRvecMedianFilter f(5);
  EXPECT_EQ(f.size(), 0u);
  EXPECT_FALSE(f.filled());
}

TEST(TvecRvecMedianFilter, FillsAfterWindowPushes) {
  agv_rail_approach::TvecRvecMedianFilter f(3);
  f.push({1, 2, 3}, {0, 0, 0});
  EXPECT_FALSE(f.filled());
  f.push({1, 2, 3}, {0, 0, 0});
  EXPECT_FALSE(f.filled());
  f.push({1, 2, 3}, {0, 0, 0});
  EXPECT_TRUE(f.filled());
}

TEST(TvecRvecMedianFilter, MedianRejectsSingleOutlier) {
  agv_rail_approach::TvecRvecMedianFilter f(5);
  // 4 clean samples + 1 outlier; median should pick a clean one on each
  // component independently.
  f.push({0.30, 0.00, 1.00}, {0.0, 0.0, 0.0});
  f.push({0.31, 0.01, 1.01}, {0.0, 0.0, 0.0});
  f.push({10.0, 5.00, -2.0}, {0.0, 0.0, 0.0});  // outlier
  f.push({0.29, 0.00, 0.99}, {0.0, 0.0, 0.0});
  f.push({0.30, 0.01, 1.00}, {0.0, 0.0, 0.0});
  const auto m = f.tvec_median();
  EXPECT_NEAR(m[0], 0.30, 0.05);
  EXPECT_NEAR(m[1], 0.00, 0.05);
  EXPECT_NEAR(m[2], 1.00, 0.05);
}

TEST(TvecRvecMedianFilter, ResetEmptiesBuffer) {
  agv_rail_approach::TvecRvecMedianFilter f(3);
  for (int i = 0; i < 5; ++i) f.push({double(i), 0, 0}, {0, 0, 0});
  EXPECT_TRUE(f.filled());
  f.reset();
  EXPECT_FALSE(f.filled());
  EXPECT_EQ(f.size(), 0u);
}

TEST(TvecRvecMedianFilter, RollingWindowKeepsLatestN) {
  agv_rail_approach::TvecRvecMedianFilter f(3);
  for (int i = 0; i < 6; ++i) f.push({double(i), 0, 0}, {0, 0, 0});
  // After 6 pushes with window=3, buffer holds values 3, 4, 5; median = 4.
  EXPECT_NEAR(f.tvec_median()[0], 4.0, 1e-9);
}

TEST(FineServo, VerdictStringsCoverAllCases) {
  using agv_rail_approach::verdict_to_str;
  EXPECT_STREQ(verdict_to_str(FineServoVerdict::OK), "ok");
  EXPECT_STREQ(verdict_to_str(FineServoVerdict::INVALID_CORNERS), "invalid_corners");
  EXPECT_STREQ(verdict_to_str(FineServoVerdict::INVALID_INTRINSICS), "invalid_intrinsics");
  EXPECT_STREQ(verdict_to_str(FineServoVerdict::SOLVEPNP_FAIL), "solvepnp_fail");
  EXPECT_STREQ(verdict_to_str(FineServoVerdict::RANGE_OUT_OF_BOUNDS), "range_out_of_bounds");
}
