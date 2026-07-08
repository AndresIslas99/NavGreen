// wheel_slip_detector — pure logic implementation.
// See header for design notes and references.

#include "agv_sensor_fusion/wheel_slip_detector.hpp"

#include <cmath>
#include <sstream>

namespace agv_sensor_fusion {

const char* to_string(SlipState s) noexcept {
  switch (s) {
    case SlipState::Inactive:    return "INACTIVE";
    case SlipState::ActiveHold:  return "ACTIVE";
    case SlipState::Settling:    return "SETTLING";
  }
  return "UNKNOWN";
}

SlipDecision WheelSlipDetector::step(const SlipObservation& obs) {
  SlipDecision dec{};
  dec.residual_yaw_rate = std::abs(obs.imu_wz - obs.wheel_wz);
  if (obs.visual_vx.has_value()) {
    dec.residual_vx = std::abs(*obs.visual_vx - obs.wheel_vx);
  }

  // Phase 2.5 — STRUCTURAL detector (Ward-Iagnemma 2008 fusion).
  // The caster dwell advisor signals when we are within the 0.5 s
  // settle window after a direction reversal. The literature
  // (Conner CMU 2001; Naveed 2023) confirms that lateral caster slip
  // during this window is invisible to instantaneous propioceptive
  // detectors (gyro vs wheel residual stays low while distance error
  // accumulates). We trust the structural signal: while DWELLING is
  // active, declare slip regardless of residuals.
  if (obs.dwell_active) {
    // Record the entry time BEFORE mutating state_: refreshing only on the
    // transition into ActiveHold keeps min_active_s counting from THIS
    // dwell event, not from a stale timestamp of an earlier slip.
    if (state_ != SlipState::ActiveHold) {
      slip_started_t_ = obs.t_now;
    }
    state_ = SlipState::ActiveHold;
    dec.state = state_;
    dec.inflate_covariance = true;
    dec.reason = "dwell_structural";
    return dec;
  }

  // Freshness gates. We accept any finite timestamp (including 0) as
  // the start of monotonic time. If the IMU is stale we cannot evaluate
  // the yaw test reliably; we hold whatever state we were in (no new
  // entry, but allow exit by settle timer). Visual freshness is only
  // meaningful when an actual visual sample exists (visual_vx set).
  const bool imu_fresh = std::isfinite(obs.t_imu_last) &&
                         (obs.t_now - obs.t_imu_last) <= p_.imu_max_age_s;
  const bool visual_fresh = obs.visual_vx.has_value() &&
                            std::isfinite(obs.t_visual_last) &&
                            (obs.t_now - obs.t_visual_last) <= p_.visual_max_age_s;

  if (!imu_fresh) {
    // No usable IMU. Don't enter slip from a missing signal, but if we
    // were already in slip, let the timers run to exit.
    dec.reason = "imu_stale";
  } else {
    const bool yaw_violates = dec.residual_yaw_rate > p_.yaw_rate_threshold_rad_s;
    const bool vx_violates = dec.residual_vx.has_value() &&
                             *dec.residual_vx > p_.linear_velocity_threshold_m_s;

    bool slip_signal = false;
    if (p_.require_visual) {
      // Both signals must agree → slip
      slip_signal = visual_fresh && yaw_violates && vx_violates;
      if (!visual_fresh) dec.reason = "visual_stale_required";
    } else {
      // Either signal is enough
      slip_signal = yaw_violates || vx_violates;
    }

    if (state_ == SlipState::Inactive) {
      if (slip_signal) {
        state_ = SlipState::ActiveHold;
        slip_started_t_ = obs.t_now;
        std::ostringstream r;
        r << (yaw_violates ? "yaw_residual" : "vx_residual");
        dec.reason = r.str();
      } else {
        dec.reason = "ok";
      }
    } else if (state_ == SlipState::ActiveHold) {
      if (slip_signal) {
        // Stay in hold; do NOT refresh slip_started_t_, so min_active_s
        // counts from the FIRST entry into slip (gives a deterministic
        // upper bound on hold time).
        dec.reason = "active_signal";
      } else if (obs.t_now - slip_started_t_ >= p_.min_active_s) {
        // Held long enough; transition to settling.
        state_ = SlipState::Settling;
        slip_cleared_t_ = obs.t_now;
        dec.reason = "active_to_settling";
      } else {
        dec.reason = "active_min_hold";
      }
    } else /* Settling */ {
      if (slip_signal) {
        // Slip came back during the settle window — re-enter hold.
        state_ = SlipState::ActiveHold;
        slip_started_t_ = obs.t_now;
        std::ostringstream r;
        r << "settling_to_active:"
          << (yaw_violates ? "yaw" : "vx");
        dec.reason = r.str();
      } else if (obs.t_now - slip_cleared_t_ >= p_.settle_s) {
        state_ = SlipState::Inactive;
        dec.reason = "settled";
      } else {
        dec.reason = "settling";
      }
    }
  }

  dec.state = state_;
  // Inflate covariance during ActiveHold AND Settling — both windows
  // are when the wheel velocity cannot be trusted.
  dec.inflate_covariance = (state_ != SlipState::Inactive);
  return dec;
}

}  // namespace agv_sensor_fusion
