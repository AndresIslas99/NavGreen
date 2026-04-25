// wheel_slip_detector — pure-logic header for the slip detector.
//
// The detector decides whether the wheel encoders' instantaneous
// velocity estimates can be trusted, by comparing them against
// independent estimates from the IMU (gyro yaw rate) and visual
// odometry (linear velocity in body frame). When either residual
// exceeds its threshold, slip is asserted; the wheel_odom updates are
// then ignored by the EKF for at least `min_active_s` seconds, with a
// `settle_s` decay window before they are trusted again.
//
// References:
//   - M. Brossard, A. Barrau, S. Bonnabel, "RINS-W: Robust Inertial
//     Navigation System on Wheels," IROS 2019.
//   - F. De Giorgi, D. De Palma, G. Parlangeli, "Online Odometry
//     Calibration for Differential Drive Mobile Robots in Low Traction
//     Conditions with Slippage," MDPI Robotics 13(1):7, 2024.
//   - robot_localization documentation, T. Moore: "Inflating
//     covariance is unnecessary and detrimental … Set the configuration
//     for the variable you'd like to ignore to false." This detector
//     follows that guidance: instead of multiplying the input
//     covariance, it raises it to a sentinel value that the EKF treats
//     as "ignore" via robot_localization's own thresholds.
//
// This header is ROS-free so the logic can be unit-tested with gtest.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace agv_sensor_fusion {

struct WheelSlipDetectorParams {
  // Hard thresholds. Above these the detector asserts slip. Defaults
  // were chosen to be larger than the typical noise floor of each
  // signal pair on the AGV at 50 Hz (wheel_odom) and 200 Hz (filtered
  // IMU); tune via docs/calibration/slip_detector_tuning.md.
  double yaw_rate_threshold_rad_s = 0.15;
  double linear_velocity_threshold_m_s = 0.05;

  // Hold + release dynamics. Once slip is asserted it stays asserted
  // for at least `min_active_s` regardless of subsequent below-
  // threshold readings (avoids chatter). After slip clears, a settle
  // window of `settle_s` keeps the inflated covariance — the caster
  // wheels physically take up to ~0.5 s to realign even after the
  // wheels match the IMU again.
  double min_active_s = 0.3;
  double settle_s = 0.5;

  // Freshness windows for the auxiliary inputs. If the IMU has not
  // updated within imu_max_age_s we cannot evaluate the yaw test;
  // same for visual. By default the detector requires the IMU but
  // tolerates missing visual (set require_visual=false to allow
  // operation without cuVSLAM).
  double imu_max_age_s = 0.1;
  double visual_max_age_s = 0.2;
  bool require_visual = false;

  // Covariance values to write into the validated odometry message.
  // `inflated_xx` is the value that signals "ignore this update" to
  // the downstream EKF (robot_localization treats >1e3 as effectively
  // ignored). `baseline_xx` is what we leave in linear-x covariance
  // when no slip is asserted; if the upstream odom already supplies a
  // sane covariance we forward it instead (controlled by
  // forward_upstream_baseline=true).
  double inflated_xx = 1.0e6;
  double baseline_xx = 0.05;       // (m/s)² — only used when forward_upstream_baseline=false
  bool forward_upstream_baseline = true;
};

enum class SlipState : std::uint8_t {
  Inactive = 0,
  ActiveHold = 1,   // slip asserted, within min_active hold
  Settling = 2,     // slip cleared, within settle window
};

const char* to_string(SlipState s) noexcept;

struct SlipObservation {
  // Inputs at the time of evaluation. NaN means "not available."
  double t_now;                     // seconds, monotonic
  double wheel_vx;                  // m/s
  double wheel_wz;                  // rad/s
  double imu_wz;                    // rad/s
  double t_imu_last;                // seconds
  std::optional<double> visual_vx;  // m/s, optional
  double t_visual_last;             // seconds, NaN if never seen
  // Phase 2.5 — Ward-Iagnemma 2008 structural fusion. The caster dwell
  // advisor knows we just executed a direction reversal; that's a
  // structural reason to suspect lateral slip even if the residuals
  // (yaw rate, vx) look clean. The diff-drive non-holonomic model
  // assumes vy=0 at the chassis, but with passive casters this fails
  // transiently after a flip. See Conner CMU 2001 + literature notes.
  bool dwell_active = false;        // true if /agv/caster/dwell_state == DWELLING
};

struct SlipDecision {
  SlipState state;
  bool inflate_covariance;          // true → write inflated_xx; false → forward baseline
  double residual_yaw_rate;         // |imu_wz - wheel_wz|
  std::optional<double> residual_vx; // |visual_vx - wheel_vx|
  std::string reason;               // human-readable, for /agv/wheel_slip/state
};

// Pure logic class — no ROS, no I/O. The node wraps it.
class WheelSlipDetector {
 public:
  explicit WheelSlipDetector(const WheelSlipDetectorParams& p) noexcept
      : p_(p), state_(SlipState::Inactive),
        slip_started_t_(0.0), slip_cleared_t_(0.0) {}

  // Evaluate one observation. Returns the decision; mutates internal
  // state machine.
  SlipDecision step(const SlipObservation& obs);

  SlipState state() const noexcept { return state_; }
  const WheelSlipDetectorParams& params() const noexcept { return p_; }

 private:
  WheelSlipDetectorParams p_;
  SlipState state_;
  double slip_started_t_;
  double slip_cleared_t_;
};

}  // namespace agv_sensor_fusion
