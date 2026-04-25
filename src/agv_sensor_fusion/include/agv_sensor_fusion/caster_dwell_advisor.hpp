// caster_dwell_advisor — pure-logic header for the caster-aware
// "dwell" advisor.
//
// The advisor watches /agv/cmd_vel for direction reversals and publishes
// an advisory state telling downstream controllers that a "dwell" of
// `dwell_s` seconds at zero velocity is recommended before the next
// reversed command, to allow passive caster wheels to physically
// realign before the chassis starts moving in the opposite direction.
//
// Caveat: this advisor does NOT mutate /agv/cmd_vel. It is a passive
// observer. Closing the loop requires either (a) a controller that
// consumes /agv/caster/dwell_state and pauses on its own (Nav2 MPPI
// custom critic, or a simple velocity_smoother extension), or (b) a
// middleware node that sits between the controller and the smoother
// and gates cmd_vel during dwell windows. Both are flagged as future
// work in docs/calibration/baseline_protocol.md.
//
// Reference: Arrizabalaga et al., "A caster-wheel-aware MPC-based
// motion planner for mobile robotics," arXiv:2110.05604 — embeds
// caster orientation in MPC state and minimizes "bore torque." This
// advisor is a reduced version that does not require caster encoders.

#pragma once

#include <cstdint>
#include <string>

namespace agv_sensor_fusion {

struct CasterDwellParams {
  // Linear velocity (m/s) below which the chassis is considered
  // "stationary." Used to detect the moment a direction change is
  // about to begin (current vx in deadband while requested vx flips
  // sign).
  double deadband_vx_m_s = 0.02;

  // How long the dwell should last after a sign change is detected.
  // Default value comes from caster_settling_tau in the previous
  // odrive_params.yaml — empirically the time the AGV's casters need
  // to fully realign on polished ceramic.
  double dwell_s = 0.5;
};

enum class DwellState : std::uint8_t {
  Idle = 0,        // current cmd_vel sign matches the recent commanded sign
  Dwelling = 1,    // a sign change has just been observed; advise pause
};

const char* to_string(DwellState s) noexcept;

struct CasterObservation {
  double t_now;           // monotonic seconds
  double cmd_vx;          // requested vx (this is the cmd_vel we observe)
  double measured_vx;     // current actual vx (from odom or feedback). Used
                          // only to know if the chassis is already at zero.
};

struct CasterAdvice {
  DwellState state;
  double seconds_remaining;   // how much longer the dwell window lasts
  double last_sign;           // sign of the last commanded direction (-1, 0, +1)
};

class CasterDwellAdvisor {
 public:
  explicit CasterDwellAdvisor(const CasterDwellParams& p) noexcept
      : p_(p), state_(DwellState::Idle), last_sign_(0.0),
        dwell_started_t_(0.0) {}

  CasterAdvice step(const CasterObservation& obs);

  DwellState state() const noexcept { return state_; }
  const CasterDwellParams& params() const noexcept { return p_; }

 private:
  static double sign_of(double v, double deadband) {
    if (v > deadband) return +1.0;
    if (v < -deadband) return -1.0;
    return 0.0;
  }

  CasterDwellParams p_;
  DwellState state_;
  double last_sign_;
  double dwell_started_t_;
};

}  // namespace agv_sensor_fusion
