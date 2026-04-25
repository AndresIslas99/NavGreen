#include <gtest/gtest.h>

#include "agv_sensor_fusion/caster_dwell_advisor.hpp"

using agv_sensor_fusion::CasterAdvice;
using agv_sensor_fusion::CasterDwellAdvisor;
using agv_sensor_fusion::CasterDwellParams;
using agv_sensor_fusion::CasterObservation;
using agv_sensor_fusion::DwellState;

namespace {
CasterObservation obs(double t, double cmd_vx, double meas_vx = 0.0) {
  return CasterObservation{t, cmd_vx, meas_vx};
}
}  // namespace

TEST(CasterDwellAdvisor, IdleWhileDirectionStable) {
  CasterDwellParams p{};
  CasterDwellAdvisor a(p);
  // Forward steady
  for (double t = 0.0; t < 2.0; t += 0.05) {
    auto adv = a.step(obs(t, +0.20));
    EXPECT_EQ(adv.state, DwellState::Idle) << "t=" << t;
  }
}

TEST(CasterDwellAdvisor, EntersDwellOnSignFlip) {
  CasterDwellParams p{};
  p.dwell_s = 0.5;
  CasterDwellAdvisor a(p);

  // Establish forward direction
  a.step(obs(0.00, +0.20));
  // Operator releases joystick — cmd zero; still Idle, last_sign=+1
  auto adv = a.step(obs(0.20, 0.0));
  EXPECT_EQ(adv.state, DwellState::Idle);
  // Operator commands reverse — sign flip → enter Dwell
  adv = a.step(obs(0.40, -0.20));
  EXPECT_EQ(adv.state, DwellState::Dwelling);
  EXPECT_GT(adv.seconds_remaining, 0.0);
}

TEST(CasterDwellAdvisor, ExitsAfterDwellSeconds) {
  CasterDwellParams p{};
  p.dwell_s = 0.5;
  CasterDwellAdvisor a(p);

  a.step(obs(0.00, +0.20));
  a.step(obs(0.20, -0.20));   // enter
  ASSERT_EQ(a.state(), DwellState::Dwelling);

  // Mid-dwell, still Dwelling
  auto adv = a.step(obs(0.40, -0.20));
  EXPECT_EQ(adv.state, DwellState::Dwelling);

  // After dwell_s elapsed, exit to Idle
  adv = a.step(obs(0.80, -0.20));
  EXPECT_EQ(adv.state, DwellState::Idle);
  EXPECT_DOUBLE_EQ(adv.last_sign, -1.0);
}

TEST(CasterDwellAdvisor, DeadbandPreventsSpuriousFlips) {
  CasterDwellParams p{};
  p.deadband_vx_m_s = 0.05;
  CasterDwellAdvisor a(p);

  // Tiny positive then tiny negative, both inside deadband — no flip.
  a.step(obs(0.00, +0.20));
  auto adv = a.step(obs(0.10, +0.02));   // inside deadband
  EXPECT_EQ(adv.state, DwellState::Idle);
  adv = a.step(obs(0.20, -0.03));        // still inside deadband
  EXPECT_EQ(adv.state, DwellState::Idle);

  // Now a real reverse command outside deadband → flip
  adv = a.step(obs(0.30, -0.20));
  EXPECT_EQ(adv.state, DwellState::Dwelling);
}
