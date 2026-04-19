// Pure controller logic for the rail_approach fine-servoing phase.
// Header-only, ROS-free — depends on OpenCV (solvePnP / Rodrigues) but
// nothing from rclcpp, tf2, geometry_msgs, or apriltag_msgs. This makes
// the control law unit-testable without spinning a node or running a
// sim iteration (a full HIL sweep is ~35 min; a gtest pass is <1 s).
//
// The node wrapper owns:
//   - camera_frame TF lookups (computes cam_to_base),
//   - the apriltag subscription + state machine,
//   - the settle_count_ counter (this controller is STATELESS; callers
//     advance / reset the counter based on `in_tolerance`).
//
// The controller owns:
//   - the solvePnP call + sanity check on the reported range,
//   - the transform from camera-optical to base_link,
//   - the error computation (x/y/yaw) and P-controller gains,
//   - the rejection taxonomy that rail_approach publishes for
//     observability (see FineServoVerdict below).

#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace agv_rail_approach {

struct FineServoParams {
  // Pinhole intrinsics (from CameraInfo.k).
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double tag_size_m = 0.2;
  // Desired pose in base_link: where the tag centre should land.
  double desired_offset_x = 0.3;
  double desired_offset_y = 0.0;
  // Convergence tolerances (position m, yaw rad).
  double tolerance_xy = 0.02;
  double tolerance_yaw = 0.017;
  // P-controller gains.
  double kp_linear = 0.15;
  double kp_lateral = 0.3;
  double kp_yaw = 0.5;
  // Velocity clamps (cmd_vel output).
  double max_linear_mps = 0.03;
  double max_angular_rps = 0.10;
  // Reject the frame if range is outside this bracket (m).
  double range_min_m = 0.05;
  double range_max_m = 5.0;
  // Include yaw in the convergence check. The yaw formula
  // (atan2(R[0][0], R[2][0]) − π) was designed for wall tags whose
  // local X axis is horizontal and whose face normal points toward
  // the camera. Floor tags (normal = world +Z) give a reference angle
  // of π under this formula, so `in_tolerance` would never latch true
  // for them. Set this false when the target tag lies on the floor
  // plane; the P-controller still computes a yaw-correction angular
  // command from the same formula, but settlement is judged on the
  // x/y errors alone.
  bool check_yaw_convergence = true;
};

enum class FineServoVerdict {
  OK,                   // corners processed; cmd is valid.
  INVALID_CORNERS,      // not exactly 4 corner points.
  INVALID_INTRINSICS,   // fx/fy <= 0 — camera_info not wired.
  SOLVEPNP_FAIL,        // OpenCV returned false.
  RANGE_OUT_OF_BOUNDS,  // ‖tvec‖ outside [range_min_m, range_max_m].
};

inline const char *verdict_to_str(FineServoVerdict v) {
  switch (v) {
    case FineServoVerdict::OK:                  return "ok";
    case FineServoVerdict::INVALID_CORNERS:     return "invalid_corners";
    case FineServoVerdict::INVALID_INTRINSICS:  return "invalid_intrinsics";
    case FineServoVerdict::SOLVEPNP_FAIL:       return "solvepnp_fail";
    case FineServoVerdict::RANGE_OUT_OF_BOUNDS: return "range_out_of_bounds";
  }
  return "unknown";
}

struct FineServoOutput {
  FineServoVerdict verdict = FineServoVerdict::OK;
  // Velocity command in base_link frame. Zero when verdict != OK.
  double cmd_linear_mps = 0.0;
  double cmd_angular_rps = 0.0;
  // Errors in base_link frame (position m, yaw rad). Signed. Zero when
  // verdict != OK.
  double error_x = 0.0;
  double error_y = 0.0;
  double error_yaw = 0.0;
  // Tag pose expressed in base_link (for viz publishers).
  double tag_x_base = 0.0, tag_y_base = 0.0, tag_z_base = 0.0;
  // Raw range from solvePnP (m). Useful for logging even when verdict
  // is RANGE_OUT_OF_BOUNDS.
  double range_m = 0.0;
  // True when ALL three errors are within their tolerances. Wrapper
  // advances settle_count_ on true, resets on false.
  bool in_tolerance = false;
};

// Step the controller once on a fresh detection.
//
// `corners` must be exactly 4 pixel points in the tag's CCW order when
// viewed from the outward face (BL, BR, TR, TL) — matches apriltag_ros.
//
// `cam_to_base` is the 4×4 homogeneous transform that maps a point in
// the camera-optical frame into base_link. Caller builds it from a TF
// lookup; passing all-zeros is undefined (caller handles TF misses
// itself — the controller does not attempt a fallback).
inline FineServoOutput fine_servo_step(
    const std::vector<cv::Point2d> &corners,
    const cv::Matx44d &cam_to_base,
    const FineServoParams &p) {
  FineServoOutput out;

  if (corners.size() != 4) {
    out.verdict = FineServoVerdict::INVALID_CORNERS;
    return out;
  }
  if (p.fx <= 0.0 || p.fy <= 0.0) {
    out.verdict = FineServoVerdict::INVALID_INTRINSICS;
    return out;
  }

  const double half = p.tag_size_m / 2.0;
  // Tag face is the local Z=0 plane. Corners in the tag's local frame
  // follow apriltag_ros's CCW-viewed-from-+Z order: BL, BR, TR, TL.
  const std::vector<cv::Point3d> obj_pts{
      {-half, -half, 0.0}, { half, -half, 0.0},
      { half,  half, 0.0}, {-half,  half, 0.0}};

  cv::Mat K = (cv::Mat_<double>(3, 3) <<
               p.fx, 0.0, p.cx,
               0.0, p.fy, p.cy,
               0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);

  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(obj_pts, corners, K, dist, rvec, tvec)) {
    out.verdict = FineServoVerdict::SOLVEPNP_FAIL;
    return out;
  }
  out.range_m = cv::norm(tvec);
  if (out.range_m < p.range_min_m || out.range_m > p.range_max_m) {
    out.verdict = FineServoVerdict::RANGE_OUT_OF_BOUNDS;
    return out;
  }

  // Transform tag origin from camera-optical into base_link (for viz).
  const cv::Vec4d tag_cam_h(tvec[0], tvec[1], tvec[2], 1.0);
  const cv::Vec4d tag_base_h = cam_to_base * tag_cam_h;
  out.tag_x_base = tag_base_h[0];
  out.tag_y_base = tag_base_h[1];
  out.tag_z_base = tag_base_h[2];

  // Extract tag yaw in camera frame, same formula as the original node:
  // the tag's apparent yaw on the camera's XZ plane. atan2(R[0][0],
  // R[2][0]) is the pre-2026 convention and the downstream maths
  // (error_yaw = tag_yaw_in_cam - π) depends on it exactly.
  cv::Mat R_ct;
  cv::Rodrigues(rvec, R_ct);
  const double tag_yaw_in_cam = std::atan2(
      R_ct.at<double>(0, 0), R_ct.at<double>(2, 0));

  // Errors are computed in CAMERA-OPTICAL frame, not base_link.
  // desired_offset_x is documented as "forward distance from CAMERA to
  // tag" in rail_approach_params.yaml; the earlier implementation
  // accidentally used base-frame, which — with the greenhouse AGV's
  // 0.70 m base→camera forward offset — would have had the controller
  // try to settle with the tag ~0.4 m BEHIND the camera (unreachable).
  // That bug is why every rail_approach in iter-7..9 stalled at
  // err_xy ≈ 0.33 m = coarse_standoff − 0.70 + desired_offset_x.
  //   camera-optical X = right (→ -base.y)
  //   camera-optical Y = down  (→ -base.z)
  //   camera-optical Z = forward (→ +base.x after adding base offset)
  // Forward error = tvec.Z - desired_offset_x.
  // Lateral error = -tvec.X - desired_offset_y   (cam X right → base Y left).
  const double tag_cam_z = tvec[2];
  const double tag_cam_x = tvec[0];
  const double error_x = tag_cam_z - p.desired_offset_x;
  const double error_y = -tag_cam_x - p.desired_offset_y;
  double error_yaw = tag_yaw_in_cam - M_PI;
  while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;

  out.error_x = error_x;
  out.error_y = error_y;
  out.error_yaw = error_yaw;

  out.cmd_linear_mps = std::clamp(
      p.kp_linear * error_x, -p.max_linear_mps, p.max_linear_mps);
  out.cmd_angular_rps = std::clamp(
      p.kp_yaw * error_yaw + p.kp_lateral * error_y,
      -p.max_angular_rps, p.max_angular_rps);

  out.in_tolerance = std::abs(error_x) < p.tolerance_xy
                     && std::abs(error_y) < p.tolerance_xy
                     && (!p.check_yaw_convergence
                         || std::abs(error_yaw) < p.tolerance_yaw);

  out.verdict = FineServoVerdict::OK;
  return out;
}

}  // namespace agv_rail_approach
