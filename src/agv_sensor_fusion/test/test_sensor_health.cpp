#include <gtest/gtest.h>
#include "agv_sensor_fusion/sensor_health.hpp"

using agv_sensor_fusion::SensorHealth;

TEST(SensorHealth, RateZeroWithNoMessages)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  EXPECT_DOUBLE_EQ(sh.rate(), 0.0);
}

TEST(SensorHealth, RateZeroWithOneMessage)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  sh.record(1.0);
  EXPECT_DOUBLE_EQ(sh.rate(), 0.0);
}

TEST(SensorHealth, RateCalculation)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  sh.window_seconds = 5.0;

  // Record 51 messages over 1 second → 50 Hz
  for (int i = 0; i <= 50; ++i) {
    sh.record(1.0 + i * 0.02);  // 20ms intervals
  }
  double r = sh.rate();
  EXPECT_NEAR(r, 50.0, 1.0);
}

TEST(SensorHealth, RateCalculationLowRate)
{
  SensorHealth sh;
  sh.expected_hz = 10.0;
  sh.window_seconds = 5.0;

  // 3 messages over 1 second → 2 Hz
  sh.record(0.0);
  sh.record(0.5);
  sh.record(1.0);
  EXPECT_NEAR(sh.rate(), 2.0, 0.1);
}

TEST(SensorHealth, AgeNoMessages)
{
  SensorHealth sh;
  EXPECT_DOUBLE_EQ(sh.age(10.0), 999.0);
}

TEST(SensorHealth, AgeWithMessages)
{
  SensorHealth sh;
  sh.record(5.0);
  EXPECT_DOUBLE_EQ(sh.age(7.0), 2.0);
}

TEST(SensorHealth, IsStaleTrue)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  sh.staleness_factor = 2.0;
  // period = 0.02s, threshold = 0.04s
  sh.record(1.0);
  EXPECT_TRUE(sh.is_stale(1.05));   // age=0.05 > 0.04
}

TEST(SensorHealth, IsStaleFalse)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  sh.staleness_factor = 2.0;
  sh.record(1.0);
  EXPECT_FALSE(sh.is_stale(1.03));  // age=0.03 < 0.04
}

TEST(SensorHealth, IsStaleEventDriven)
{
  // Event-driven sensors (expected_hz=0) should never be stale
  SensorHealth sh;
  sh.expected_hz = 0.0;
  EXPECT_FALSE(sh.is_stale(999.0));
}

TEST(SensorHealth, IsRateLowTrue)
{
  SensorHealth sh;
  sh.expected_hz = 50.0;
  sh.window_seconds = 5.0;

  // Record 5 messages over 1 second → ~4 Hz (< 25 Hz threshold)
  for (int i = 0; i < 5; ++i) {
    sh.record(1.0 + i * 0.25);
  }
  EXPECT_TRUE(sh.is_rate_low(2.0));
}

TEST(SensorHealth, IsRateLowFalse)
{
  SensorHealth sh;
  sh.expected_hz = 10.0;
  sh.window_seconds = 5.0;

  // Record 11 messages over 1 second → 10 Hz (>= 5 Hz threshold)
  for (int i = 0; i <= 10; ++i) {
    sh.record(1.0 + i * 0.1);
  }
  EXPECT_FALSE(sh.is_rate_low(2.0));
}

TEST(SensorHealth, WindowPruning)
{
  SensorHealth sh;
  sh.expected_hz = 10.0;
  sh.window_seconds = 2.0;

  // Record at t=0,1,2,3,4,5
  for (int i = 0; i <= 5; ++i) {
    sh.record(static_cast<double>(i));
  }
  // Window is 2s, so at t=5 only t=3,4,5 should remain
  EXPECT_LE(sh.timestamps.size(), 4u);
  EXPECT_EQ(sh.total_count, 6u);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
