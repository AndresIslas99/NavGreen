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
#include <deque>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace agv_rail_approach {

// ── Median filter for solvePnP output ─────────────────────────────────
//
// solvePnP on a floor tag viewed at ~80° of incidence produces cm-level
// jitter on tvec between successive frames (narrow trapezoidal corner
// projection amplifies corner-localization noise). The node pushes each
// fresh {tvec, rvec} pair into this filter; once the rolling window is
// full the output is the element-wise median across the last N pairs.
// Median is chosen over mean to reject single-frame outliers (e.g. a
// corner that clipped the frame edge) without phase lag.
//
// The filter is header-only + ROS-free so it is exercised directly by
// test_fine_servo_controller.cpp (TvecRvecMedianFilter.* cases).
class TvecRvecMedianFilter {
 public:
  explicit TvecRvecMedianFilter(std::size_t window = 5) : window_(window) {
    if (window_ == 0) window_ = 1;
  }

  void push(const cv::Vec3d &tvec, const cv::Vec3d &rvec) {
    tvecs_.push_back(tvec);
    rvecs_.push_back(rvec);
    while (tvecs_.size() > window_) tvecs_.pop_front();
    while (rvecs_.size() > window_) rvecs_.pop_front();
  }

  void reset() {
    tvecs_.clear();
    rvecs_.clear();
  }

  std::size_t size() const { return tvecs_.size(); }
  bool filled() const { return tvecs_.size() >= window_; }

  // Median of each component independently. Uses a copy + nth_element
  // so the filter itself stays O(N log N) per call with N = window ≤ 5.
  cv::Vec3d tvec_median() const { return axiswise_median(tvecs_); }
  cv::Vec3d rvec_median() const { return axiswise_median(rvecs_); }

 private:
  static cv::Vec3d axiswise_median(const std::deque<cv::Vec3d> &d) {
    cv::Vec3d out(0.0, 0.0, 0.0);
    if (d.empty()) return out;
    std::vector<double> xs, ys, zs;
    xs.reserve(d.size());
    ys.reserve(d.size());
    zs.reserve(d.size());
    for (const auto &v : d) {
      xs.push_back(v[0]);
      ys.push_back(v[1]);
      zs.push_back(v[2]);
    }
    auto mid = xs.size() / 2;
    std::nth_element(xs.begin(), xs.begin() + mid, xs.end());
    std::nth_element(ys.begin(), ys.begin() + mid, ys.end());
    std::nth_element(zs.begin(), zs.begin() + mid, zs.end());
    // nth_element places the mid-th element in position — for a true
    // median on even-sized windows we'd average with the one below,
    // but the controller tolerates ≤1 sample bias at window sizes
    // ≥ 3, so skip the extra sort for speed.
    out[0] = xs[mid];
    out[1] = ys[mid];
    out[2] = zs[mid];
    return out;
  }

  std::size_t window_;
  std::deque<cv::Vec3d> tvecs_;
  std::deque<cv::Vec3d> rvecs_;
};

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

// Pure-math half of the servo step: given a valid tvec/rvec pair
// (already produced by solvePnP and optionally smoothed through a
// median filter), compute the cmd/error fields + convergence flag.
// Does no solvePnP, no TF, no logging. Callable from unit tests with
// hand-crafted inputs.
inline FineServoOutput fine_servo_compute(
    const cv::Vec3d &tvec,
    const cv::Vec3d &rvec,
    const cv::Matx44d &cam_to_base,
    const FineServoParams &p) {
  FineServoOutput out;
  if (p.fx <= 0.0 || p.fy <= 0.0) {
    out.verdict = FineServoVerdict::INVALID_INTRINSICS;
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
  // Iter-42.1: when check_yaw_convergence is false (floor tag), the yaw
  // reference of π is a singular point of the atan2-based tag_yaw_in_cam
  // formula — R[0,0] and R[2,0] are mathematically zero + one but
  // floating-point noise tilts the wrap one side or the other of −π,
  // which flips the sign of kp_yaw*error_yaw and makes the angular
  // command non-deterministic across solvePnP method choice (ITERATIVE
  // happened to land on the +ε side, SQPNP on the −ε side, failing the
  // LateralOffsetProducesAngularCmd test). Zero the yaw contribution
  // for floor tags so lateral correction relies solely on error_y; that
  // is the semantically correct behaviour since a floor tag's in-plane
  // yaw is independent of the robot's heading-to-tag line. Wall tags
  // keep the full coupling (they genuinely rotate the tag's own frame
  // relative to the camera).
  const double yaw_contrib =
      p.check_yaw_convergence ? (p.kp_yaw * error_yaw) : 0.0;
  out.cmd_angular_rps = std::clamp(
      yaw_contrib + p.kp_lateral * error_y,
      -p.max_angular_rps, p.max_angular_rps);

  out.in_tolerance = std::abs(error_x) < p.tolerance_xy
                     && std::abs(error_y) < p.tolerance_xy
                     && (!p.check_yaw_convergence
                         || std::abs(error_yaw) < p.tolerance_yaw);

  out.verdict = FineServoVerdict::OK;
  return out;
}

// Convenience wrapper: solvePnP on the 4 tag corners, then forward the
// tvec/rvec into fine_servo_compute. No median filter — call-sites
// that want smoothing should use solvepnp_tag() + push into a
// TvecRvecMedianFilter themselves, then call fine_servo_compute with
// the filtered tvec/rvec.
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
  const std::vector<cv::Point3d> obj_pts{
      {-half, -half, 0.0}, { half, -half, 0.0},
      { half,  half, 0.0}, {-half,  half, 0.0}};
  cv::Mat K = (cv::Mat_<double>(3, 3) <<
               p.fx, 0.0, p.cx,
               0.0, p.fy, p.cy,
               0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
  cv::Vec3d rvec, tvec;
  // Iter-42 empirical benchmark (tools/solvepnp_noise_benchmark.py,
  // 1000 Monte-Carlo per method): default SOLVEPNP_ITERATIVE yields a
  // 0.8 % planar-flip rate in our realistic scenario (cam z=0.21, incidence
  // 79°) and diverges entirely in grazing (cam z=0.01, σ_z = 23 M meters).
  // SOLVEPNP_SQPNP (Terzakis & Lourakis ECCV 2020) has 0 % flips in both
  // scenarios and σ matching ITERATIVE in the good case, while remaining
  // stable at 26 mm σ even in grazing. SOLVEPNP_IPPE_SQUARE returns 100 %
  // flipped poses under the simple solvePnP API (it needs
  // solvePnPGeneric + reprojection disambiguation to be correct). SQPNP
  // is the drop-in winner.
  if (!cv::solvePnP(obj_pts, corners, K, dist, rvec, tvec,
                     false, cv::SOLVEPNP_SQPNP)) {
    out.verdict = FineServoVerdict::SOLVEPNP_FAIL;
    return out;
  }
  return fine_servo_compute(tvec, rvec, cam_to_base, p);
}

// Helper for call-sites that want to apply a median filter between
// solvePnP and the control computation. Returns false on
// invalid_corners / invalid_intrinsics / solvepnp_fail; sets out_reason
// to the matching verdict string for the caller's status publisher.
inline bool solvepnp_tag(
    const std::vector<cv::Point2d> &corners,
    const FineServoParams &p,
    cv::Vec3d &tvec,
    cv::Vec3d &rvec,
    FineServoVerdict &out_verdict) {
  if (corners.size() != 4) {
    out_verdict = FineServoVerdict::INVALID_CORNERS;
    return false;
  }
  if (p.fx <= 0.0 || p.fy <= 0.0) {
    out_verdict = FineServoVerdict::INVALID_INTRINSICS;
    return false;
  }
  const double half = p.tag_size_m / 2.0;
  const std::vector<cv::Point3d> obj_pts{
      {-half, -half, 0.0}, { half, -half, 0.0},
      { half,  half, 0.0}, {-half,  half, 0.0}};
  cv::Mat K = (cv::Mat_<double>(3, 3) <<
               p.fx, 0.0, p.cx,
               0.0, p.fy, p.cy,
               0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
  // Iter-42: SOLVEPNP_SQPNP (see fine_servo_step comment above for the
  // empirical-benchmark rationale). Zero flips, σ matches the default
  // iterative solver in the good case and stays stable in grazing.
  if (!cv::solvePnP(obj_pts, corners, K, dist, rvec, tvec,
                     false, cv::SOLVEPNP_SQPNP)) {
    out_verdict = FineServoVerdict::SOLVEPNP_FAIL;
    return false;
  }
  out_verdict = FineServoVerdict::OK;
  return true;
}

}  // namespace agv_rail_approach
