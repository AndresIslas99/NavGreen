// caster_dwell_advisor — pure-logic implementation.

#include "agv_sensor_fusion/caster_dwell_advisor.hpp"

#include <cmath>

namespace agv_sensor_fusion {

const char* to_string(DwellState s) noexcept {
  switch (s) {
    case DwellState::Idle:     return "IDLE";
    case DwellState::Dwelling: return "DWELLING";
  }
  return "UNKNOWN";
}

CasterAdvice CasterDwellAdvisor::step(const CasterObservation& obs) {
  CasterAdvice a{};
  const double cmd_sign = sign_of(obs.cmd_vx, p_.deadband_vx_m_s);

  // Detect a direction change: previously had a non-zero sign, now
  // commanding the opposite non-zero sign. We only assert dwell at
  // the actual flip; commanding zero between the two flips is fine
  // and the operator's natural soft-stop already gives us part of
  // the dwell.
  if (state_ == DwellState::Idle) {
    if (last_sign_ != 0.0 && cmd_sign != 0.0 &&
        cmd_sign * last_sign_ < 0.0) {
      state_ = DwellState::Dwelling;
      dwell_started_t_ = obs.t_now;
    }
    if (cmd_sign != 0.0) last_sign_ = cmd_sign;
  } else /* Dwelling */ {
    if (obs.t_now - dwell_started_t_ >= p_.dwell_s) {
      state_ = DwellState::Idle;
      // Latch the new direction
      if (cmd_sign != 0.0) last_sign_ = cmd_sign;
    }
  }

  a.state = state_;
  a.last_sign = last_sign_;
  a.seconds_remaining = (state_ == DwellState::Dwelling)
      ? std::max(0.0, p_.dwell_s - (obs.t_now - dwell_started_t_))
      : 0.0;
  return a;
}

}  // namespace agv_sensor_fusion
