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
using agv_rail_approach::FineServoState;
using agv_rail_approach::FineServoVerdict;
using agv_rail_approach::fine_servo_compute;
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


// ── Iter-46 Paso 1.b: PI + stiction feedforward ───────────────────────
//
// The tests below use fine_servo_compute directly with hand-crafted
// tvec/rvec values so the controller state trajectory is deterministic
// across ticks. For the forward axis we need:
//
//   error_x = tvec.z - desired_offset_x
//
// so tvec.z controls error_x directly. A tag at cam_z = 0.30 + E gives
// error_x = E. All yaw math is bypassed (check_yaw_convergence = false,
// floor tag), and rvec is set to the identity rotation for the floor
// tag normal — any R_ct with R[0,0] = 1 makes tag_yaw_in_cam = 0.
//
// The rvec below produces a rotation matrix whose (0,0) = 1 and (2,0) =
// 0, so tag_yaw_in_cam = atan2(1, 0) = π/2, and error_yaw = π/2 − π =
// -π/2. We keep the yaw check disabled so that does not matter.

namespace {
// Rotation (in Rodrigues form) corresponding to a floor tag facing up.
// Any fixed value that produces a valid rotation matrix is fine — we
// disable yaw-convergence so the rvec choice does not influence the
// control law beyond guarding against NaN.
const cv::Vec3d kFloorTagRvec(0.0, 0.0, 0.0);

FineServoParams pi_ff_params() {
  auto p = default_params();
  // Match the HIL iter-46 Paso 1.b configuration.
  p.tolerance_xy = 0.05;
  p.kp_linear = 0.15;
  p.ki_linear = 0.05;
  p.stiction_ff_vel_mps = 0.035;
  p.max_linear_mps = 0.30;
  p.check_yaw_convergence = false;
  return p;
}

cv::Vec3d tag_tvec(double cam_z, double cam_x = 0.0, double cam_y = 0.21) {
  return cv::Vec3d(cam_x, cam_y, cam_z);
}
}  // namespace

TEST(FineServoPI, PureP_NoIntegration_WhenKiIsZero) {
  auto p = default_params();
  p.ki_linear = 0.0;
  p.stiction_ff_vel_mps = 0.0;
  FineServoState state;
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX + 0.10), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  EXPECT_NEAR(out.cmd_linear_mps, p.kp_linear * 0.10, 1e-9);
  // State must not be mutated when ki is zero.
  EXPECT_EQ(state.integral_x, 0.0);
  EXPECT_FALSE(state.has_last_error);
}

TEST(FineServoPI, IntegralAccumulatesInPIMode) {
  auto p = pi_ff_params();
  // Force PI mode (not FF): pick |error_x| small enough to be in
  // PI regime but large enough that it does not hit FF (FF triggers
  // when |pi_cmd| < stiction_ff).
  // At error_x = 0.40, P alone = 0.15 * 0.40 = 0.06 > 0.035 → FF off.
  FineServoState state;
  for (int i = 0; i < 5; ++i) {
    fine_servo_compute(
        tag_tvec(kDesiredOffsetX + 0.40), kFloorTagRvec,
        greenhouse_cam_to_base(), p, state, 0.2);
  }
  // After 5 ticks @ 0.2 s with error_x = 0.40: integral = 5 * 0.2 * 0.40 = 0.40 m·s.
  // Anti-windup clamp: 0.25 * max_linear_mps / ki = 0.25 * 0.30 / 0.05 = 1.5
  // → 0.40 is below the clamp, so the raw accumulation survives.
  EXPECT_NEAR(state.integral_x, 0.40, 1e-6);
}

TEST(FineServoPI, FFActivatesBelowStictionThreshold) {
  auto p = pi_ff_params();
  FineServoState state;
  // Pick error_x so PI command < stiction_ff (0.035) and |error_x| > tol.
  // error_x = 0.10 → P-cmd = 0.015, integral=0 on first tick → pi_cmd = 0.015 < 0.035.
  // FF should fire with sign(error) = +.
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX + 0.10), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  EXPECT_NEAR(out.cmd_linear_mps, +p.stiction_ff_vel_mps, 1e-9);
  // While FF is active, the integrator must NOT accumulate
  // (conditional integration).
  EXPECT_EQ(state.integral_x, 0.0);
}

TEST(FineServoPI, FFInhibitedInsideTolerance) {
  auto p = pi_ff_params();
  FineServoState state;
  // |error_x| = 0.03 < tolerance_xy = 0.05 → FF must NOT fire.
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX + 0.03), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  // Cmd = P + I. P-term = 0.15 * 0.03 = 0.0045. Integral accumulated this
  // tick = 0.03 * 0.2 = 0.006 m·s → I-term = 0.05 * 0.006 = 0.0003.
  // Total cmd = 0.0048 (well under stiction, robot will not move in HIL).
  EXPECT_NEAR(out.cmd_linear_mps, 0.0048, 1e-9);
  EXPECT_NEAR(state.integral_x, 0.006, 1e-9);
}

TEST(FineServoPI, ZeroCrossResetsIntegral) {
  auto p = pi_ff_params();
  FineServoState state;
  // Accumulate positive integral over several ticks at positive error.
  for (int i = 0; i < 3; ++i) {
    fine_servo_compute(
        tag_tvec(kDesiredOffsetX + 0.40), kFloorTagRvec,
        greenhouse_cam_to_base(), p, state, 0.2);
  }
  ASSERT_GT(state.integral_x, 0.0);
  // Now flip sign: robot crossed goal to the other side.
  // error_x = -0.40 and |pi_cmd| = |Kp*(-0.40) + Ki*integral| = |-0.06 + +ve| may
  // still exceed stiction; regardless, the zero-crossing reset fires BEFORE
  // the integration step, so integral_x is set to 0, then accumulates -0.08.
  fine_servo_compute(
      tag_tvec(kDesiredOffsetX - 0.40), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  EXPECT_NEAR(state.integral_x, -0.08, 1e-9);
}

TEST(FineServoPI, AntiWindupClampsIntegral) {
  auto p = pi_ff_params();
  FineServoState state;
  // Hold a large positive error for many ticks so the integrator would
  // grow unbounded without the clamp. Anti-windup cap for this config:
  //   max = 0.25 * max_linear_mps / ki = 0.25 * 0.30 / 0.05 = 1.5
  // We accumulate 500 ticks * 0.2 * 0.40 = 40.0 m·s nominal, must clamp to 1.5.
  for (int i = 0; i < 500; ++i) {
    fine_servo_compute(
        tag_tvec(kDesiredOffsetX + 0.40), kFloorTagRvec,
        greenhouse_cam_to_base(), p, state, 0.2);
  }
  EXPECT_NEAR(state.integral_x, 1.5, 1e-6);
}

TEST(FineServoPI, FFUsesErrorSignForReverse) {
  auto p = pi_ff_params();
  FineServoState state;
  // Negative error (robot past the goal) with small magnitude so FF fires.
  // error_x = -0.10 → P-cmd = -0.015, |pi| < stiction → FF: -0.035.
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX - 0.10), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  EXPECT_NEAR(out.cmd_linear_mps, -p.stiction_ff_vel_mps, 1e-9);
}

TEST(FineServoPI, FirstTickWithZeroDtSkipsIntegration) {
  auto p = pi_ff_params();
  FineServoState state;
  // Force PI mode (not FF): error_x = 0.50 → P = 0.075 > stiction.
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX + 0.50), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.0 /* first tick: dt = 0 */);
  // No integration on dt=0, so cmd = P-only.
  EXPECT_NEAR(out.cmd_linear_mps, 0.075, 1e-9);
  EXPECT_EQ(state.integral_x, 0.0);
  // But last_error_x is still tracked (needed for zero-cross detection
  // across the first pair of ticks).
  EXPECT_TRUE(state.has_last_error);
  EXPECT_NEAR(state.last_error_x, 0.50, 1e-9);
}

TEST(FineServoPI, ResetClearsEverything) {
  FineServoState state;
  state.integral_x = 1.23;
  state.last_error_x = 0.45;
  state.has_last_error = true;
  state.reset();
  EXPECT_EQ(state.integral_x, 0.0);
  EXPECT_EQ(state.last_error_x, 0.0);
  EXPECT_FALSE(state.has_last_error);
}

TEST(FineServoPI, CmdClampedAtMaxLinearMps) {
  auto p = pi_ff_params();
  FineServoState state;
  // Pick a tag distance that stays inside FineServoParams::range_max_m
  // (5 m) so the frame isn't rejected as RANGE_OUT_OF_BOUNDS. At cam_z
  // = 3.0 the error_x = 2.7, P-term alone = 0.15 * 2.7 = 0.405, above
  // the 0.30 clamp. Verify the output clamps cleanly.
  auto out = fine_servo_compute(
      tag_tvec(kDesiredOffsetX + 2.7), kFloorTagRvec,
      greenhouse_cam_to_base(), p, state, 0.2);
  ASSERT_EQ(out.verdict, FineServoVerdict::OK);
  EXPECT_NEAR(out.cmd_linear_mps, p.max_linear_mps, 1e-9);
}
