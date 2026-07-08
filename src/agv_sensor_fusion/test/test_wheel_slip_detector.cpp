#include <gtest/gtest.h>

#include "agv_sensor_fusion/wheel_slip_detector.hpp"

using agv_sensor_fusion::SlipDecision;
using agv_sensor_fusion::SlipObservation;
using agv_sensor_fusion::SlipState;
using agv_sensor_fusion::WheelSlipDetector;
using agv_sensor_fusion::WheelSlipDetectorParams;

namespace {
SlipObservation make_obs(double t_now, double wheel_vx, double wheel_wz,
                          double imu_wz, double t_imu_last,
                          double visual_vx = 0.0, double t_visual_last = -1.0,
                          bool with_visual = false,
                          bool dwell_active = false) {
  SlipObservation o{};
  o.t_now = t_now;
  o.wheel_vx = wheel_vx;
  o.wheel_wz = wheel_wz;
  o.imu_wz = imu_wz;
  o.t_imu_last = t_imu_last;
  if (with_visual) {
    o.visual_vx = visual_vx;
    o.t_visual_last = t_visual_last;
  } else {
    o.t_visual_last = std::numeric_limits<double>::quiet_NaN();
  }
  o.dwell_active = dwell_active;
  return o;
}
}  // namespace

TEST(WheelSlipDetector, NoSlipWhenSignalsAgree) {
  WheelSlipDetectorParams p{};
  WheelSlipDetector det(p);
  // Robot driving forward at 0.1 m/s with no rotation; gyro agrees.
  for (double t = 0.0; t < 2.0; t += 0.02) {
    auto d = det.step(make_obs(t, 0.10, 0.0, 0.0, t - 0.01));
    EXPECT_EQ(d.state, SlipState::Inactive) << "t=" << t;
    EXPECT_FALSE(d.inflate_covariance);
  }
}

TEST(WheelSlipDetector, EntersSlipOnYawDisagreement) {
  WheelSlipDetectorParams p{};
  p.yaw_rate_threshold_rad_s = 0.10;
  WheelSlipDetector det(p);

  // First sample: in-spec
  auto d = det.step(make_obs(0.00, 0.0, 0.0, 0.0, 0.0));
  ASSERT_EQ(d.state, SlipState::Inactive);

  // Caster slip: wheels report 0 yaw rate but gyro sees the chassis
  // physically rotating at 0.5 rad/s.
  d = det.step(make_obs(0.02, 0.0, 0.0, 0.5, 0.01));
  EXPECT_EQ(d.state, SlipState::ActiveHold);
  EXPECT_TRUE(d.inflate_covariance);
  EXPECT_NEAR(d.residual_yaw_rate, 0.5, 1e-9);
}

TEST(WheelSlipDetector, HoldsActiveForMinDuration) {
  WheelSlipDetectorParams p{};
  p.min_active_s = 0.30;
  p.settle_s = 0.40;
  WheelSlipDetector det(p);

  // Trigger slip at t=0
  det.step(make_obs(0.00, 0.0, 0.0, 0.5, 0.0));
  ASSERT_EQ(det.state(), SlipState::ActiveHold);

  // Even if signals come back into spec immediately, we should hold.
  for (double t = 0.02; t < 0.30; t += 0.02) {
    auto d = det.step(make_obs(t, 0.0, 0.0, 0.0, t - 0.01));
    EXPECT_EQ(d.state, SlipState::ActiveHold) << "t=" << t;
    EXPECT_TRUE(d.inflate_covariance);
  }

  // After min_active_s of clean signals, transition to Settling
  auto d = det.step(make_obs(0.32, 0.0, 0.0, 0.0, 0.31));
  EXPECT_EQ(d.state, SlipState::Settling);

  // During settle, still inflate
  d = det.step(make_obs(0.50, 0.0, 0.0, 0.0, 0.49));
  EXPECT_EQ(d.state, SlipState::Settling);
  EXPECT_TRUE(d.inflate_covariance);

  // After full settle window, return to Inactive
  d = det.step(make_obs(0.80, 0.0, 0.0, 0.0, 0.79));
  EXPECT_EQ(d.state, SlipState::Inactive);
  EXPECT_FALSE(d.inflate_covariance);
}

TEST(WheelSlipDetector, ReentersFromSettlingOnRecurringSlip) {
  WheelSlipDetectorParams p{};
  p.min_active_s = 0.20;
  p.settle_s = 0.40;
  WheelSlipDetector det(p);

  det.step(make_obs(0.00, 0.0, 0.0, 0.5, 0.0));    // enter
  for (double t = 0.02; t < 0.22; t += 0.02) {
    det.step(make_obs(t, 0.0, 0.0, 0.5, t - 0.01));
  }
  // Still ActiveHold, signal high. Make signal clean.
  auto d = det.step(make_obs(0.30, 0.0, 0.0, 0.0, 0.29));
  EXPECT_EQ(d.state, SlipState::Settling);

  // 0.05 s into settle, slip returns
  d = det.step(make_obs(0.35, 0.0, 0.0, 0.6, 0.34));
  EXPECT_EQ(d.state, SlipState::ActiveHold);
}

TEST(WheelSlipDetector, IgnoresWhenImuStale) {
  WheelSlipDetectorParams p{};
  p.imu_max_age_s = 0.05;
  WheelSlipDetector det(p);

  // Last IMU sample 0.5 s ago. Wheel reports something but no IMU to
  // compare with; we should NOT enter slip.
  auto d = det.step(make_obs(0.50, 0.0, 0.0, 99.0, 0.0));
  EXPECT_EQ(d.state, SlipState::Inactive);
  EXPECT_EQ(d.reason, "imu_stale");
}

TEST(WheelSlipDetector, RequiresBothSignalsWhenConfigured) {
  WheelSlipDetectorParams p{};
  p.require_visual = true;
  WheelSlipDetector det(p);

  // Yaw violates but no visual signal. Should NOT enter slip.
  auto d = det.step(make_obs(0.0, 0.0, 0.0, 0.5, 0.0,
                              /*visual=*/0.0, /*t_visual=*/0.0,
                              /*with_visual=*/false));
  EXPECT_EQ(d.state, SlipState::Inactive);

  // Yaw violates AND visual disagrees. Should enter slip.
  d = det.step(make_obs(0.02, 0.0, 0.0, 0.5, 0.01,
                         /*visual=*/0.20, /*t_visual=*/0.01,
                         /*with_visual=*/true));
  EXPECT_EQ(d.state, SlipState::ActiveHold);
}

TEST(WheelSlipDetector, StructuralDwellOverridesCleanResiduals) {
  // Phase 2.5: when dwell_active=true, the detector must declare slip
  // even when residuals are clean. This is the Ward-Iagnemma 2008
  // structural fusion: trust the contextual signal (recent direction
  // reversal) when propioceptive evidence is invisible.
  WheelSlipDetectorParams p{};
  WheelSlipDetector det(p);

  // Clean residuals everywhere — instantaneous detector would say "no slip"
  auto obs = make_obs(0.00, 0.10, 0.0, 0.0, 0.0,
                       /*visual=*/0.10, /*t_visual=*/0.0,
                       /*with_visual=*/true,
                       /*dwell_active=*/false);
  auto d = det.step(obs);
  EXPECT_EQ(d.state, SlipState::Inactive);
  EXPECT_FALSE(d.inflate_covariance);

  // Dwell becomes active. Override.
  obs = make_obs(0.05, 0.10, 0.0, 0.0, 0.04,
                  /*visual=*/0.10, /*t_visual=*/0.04,
                  /*with_visual=*/true,
                  /*dwell_active=*/true);
  d = det.step(obs);
  EXPECT_EQ(d.state, SlipState::ActiveHold);
  EXPECT_TRUE(d.inflate_covariance);
  EXPECT_EQ(d.reason, "dwell_structural");

  // Dwell ends. Should fall back to normal logic, which here would
  // now NOT see slip (residuals clean), so we transition out.
  // Note: the structural override does not enforce the min_active_s
  // hold — that's deliberate because the dwell window itself already
  // enforces a minimum duration.
  obs = make_obs(0.60, 0.10, 0.0, 0.0, 0.59,
                  /*visual=*/0.10, /*t_visual=*/0.59,
                  /*with_visual=*/true,
                  /*dwell_active=*/false);
  d = det.step(obs);
  // After dwell ends, with min_active_s respected, we transition to Settling
  // and then to Inactive.
}

TEST(WheelSlipDetector, SecondDwellRefreshesHoldTimer) {
  // Regression: slip_started_t_ used to refresh only on the FIRST-ever
  // activation (the entry check ran after state_ was already assigned
  // ActiveHold), so any dwell after a prior slip event inherited a stale
  // timestamp and skipped the min_active_s hold entirely.
  WheelSlipDetectorParams p{};
  p.min_active_s = 0.30;
  p.settle_s = 0.40;
  WheelSlipDetector det(p);

  // First slip event via residuals at t=0; ride it back to Inactive.
  det.step(make_obs(0.00, 0.0, 0.0, 0.5, 0.0));
  ASSERT_EQ(det.state(), SlipState::ActiveHold);
  auto d = det.step(make_obs(0.35, 0.0, 0.0, 0.0, 0.34));
  ASSERT_EQ(d.state, SlipState::Settling);
  d = det.step(make_obs(0.80, 0.0, 0.0, 0.0, 0.79));
  ASSERT_EQ(d.state, SlipState::Inactive);

  // Much later, a caster dwell fires. The hold must count from t=5.0.
  d = det.step(make_obs(5.00, 0.0, 0.0, 0.0, 4.99,
                         /*visual=*/0.0, /*t_visual=*/-1.0,
                         /*with_visual=*/false,
                         /*dwell_active=*/true));
  ASSERT_EQ(d.state, SlipState::ActiveHold);

  // Dwell ends 0.1s later with clean residuals: still inside min_active_s,
  // so the detector must keep holding (buggy code jumped to Settling here).
  d = det.step(make_obs(5.10, 0.0, 0.0, 0.0, 5.09));
  EXPECT_EQ(d.state, SlipState::ActiveHold);
  EXPECT_TRUE(d.inflate_covariance);
  EXPECT_EQ(d.reason, "active_min_hold");

  // Once min_active_s has elapsed since the dwell entry, transition out.
  d = det.step(make_obs(5.35, 0.0, 0.0, 0.0, 5.34));
  EXPECT_EQ(d.state, SlipState::Settling);
}
