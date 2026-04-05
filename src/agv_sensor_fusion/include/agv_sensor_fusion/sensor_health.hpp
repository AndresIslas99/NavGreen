#ifndef AGV_SENSOR_FUSION__SENSOR_HEALTH_HPP_
#define AGV_SENSOR_FUSION__SENSOR_HEALTH_HPP_

#include <deque>
#include <string>
#include <cstdint>

namespace agv_sensor_fusion {

/**
 * Tracks health metrics for a single sensor source: message rate,
 * staleness, and total message count over a rolling time window.
 *
 * Uses a simple timestamp model (double seconds) so the struct can be
 * tested without ROS time dependencies.
 */
struct SensorHealth {
  std::string name;
  double expected_hz{0.0};
  double staleness_factor{2.0};
  double last_time_s{-1.0};
  std::deque<double> timestamps;
  double window_seconds{5.0};
  uint64_t total_count{0};

  double rate() const {
    if (timestamps.size() < 2) return 0.0;
    double span = timestamps.back() - timestamps.front();
    if (span <= 0.0) return 0.0;
    return static_cast<double>(timestamps.size() - 1) / span;
  }

  void record(double now_s) {
    last_time_s = now_s;
    timestamps.push_back(now_s);
    total_count++;
    while (!timestamps.empty() &&
           (now_s - timestamps.front()) > window_seconds) {
      timestamps.pop_front();
    }
  }

  double age(double now_s) const {
    if (total_count == 0) return 999.0;
    return now_s - last_time_s;
  }

  bool is_stale(double now_s) const {
    if (expected_hz <= 0.0) return false;
    double period = 1.0 / expected_hz;
    return age(now_s) > period * staleness_factor;
  }

  bool is_rate_low(double /*now_s*/) const {
    return total_count > 2 && rate() < expected_hz * 0.5;
  }
};

}  // namespace agv_sensor_fusion

#endif  // AGV_SENSOR_FUSION__SENSOR_HEALTH_HPP_
