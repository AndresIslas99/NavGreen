// Pure controller logic for the rail driver. Header-only, no ROS deps.
// Enables unit testing of the control law without spinning a node.

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace agv_rail_driver {

struct RailControllerParams {
  double kP = 1.0;                    // Proportional gain on longitudinal error
  double speed_max_mps = 1.0;         // Absolute cap on commanded linear velocity
  double stop_band_m = 0.05;          // |err_x| below this → state=reached, cmd=0
  double lateral_abort_m = 0.30;      // |y - goal_y| exceeding this → abort
  double yaw_abort_rad = 0.26;        // |rail_yaw_error| exceeding this in rail → abort
                                      // (0.26 rad ≈ 15°; robot geometry tolerates ~36°
                                      //  but we stop well short of contact).
};

enum class RailState {
  IDLE,                // No goal
  DRIVING,             // Actively driving toward goal
  REACHED,             // Within stop_band; cmd=0; publishes state then idles
  BLOCKED_WAIT,        // collision_monitor halted us; holding cmd=0
  BLOCKED_MISALIGNED,  // Yaw-from-rail too large to safely move
  BLOCKED_LATERAL,     // Lateral drift exceeded lateral_abort_m
};

inline const char *state_to_str(RailState s) {
  switch (s) {
    case RailState::IDLE:               return "idle";
    case RailState::DRIVING:            return "driving";
    case RailState::REACHED:            return "reached";
    case RailState::BLOCKED_WAIT:       return "blocked_wait";
    case RailState::BLOCKED_MISALIGNED: return "blocked_misaligned";
    case RailState::BLOCKED_LATERAL:    return "blocked_lateral";
  }
  return "unknown";
}

struct RailControllerInputs {
  // Pose (map frame)
  double current_x = 0.0;
  double current_y = 0.0;
  // Goal (map frame) — rail axis is +X, so goal_x/goal_y are the longitudinal
  // target and the expected lateral line, respectively.
  double goal_x = 0.0;
  double goal_y = 0.0;
  // Rail axis sign: +1 = drive toward +X (forward), -1 = drive toward -X (reverse).
  // The sign should match (goal_x - current_x)'s sign; mismatches mean "already
  // past the goal" and the controller holds still.
  double rail_axis_sign = 1.0;
  // From zone_detector: rail_yaw_error is robot heading vs rail axis, NaN if
  // not in a rail aisle. For gap-zone drives, pass yaw directly instead.
  double rail_yaw_error = 0.0;
  // Gating flags
  bool   in_rail_zone = false;           // true → enforce yaw_abort check
  bool   collision_monitor_stop = false; // halt signal from safety chain
  bool   have_goal = false;              // explicit flag — IDLE without a goal
};

struct RailControllerOutput {
  double linear_x = 0.0;   // m/s, robot frame
  double angular_z = 0.0;  // rad/s — ALWAYS 0 in this controller
  RailState state = RailState::IDLE;
  double remaining_m = 0.0;
};

inline RailControllerOutput compute(const RailControllerInputs &in,
                                    const RailControllerParams &p) {
  RailControllerOutput out;
  out.angular_z = 0.0;  // Hardcoded; the entire point of this controller.

  if (!in.have_goal) {
    out.state = RailState::IDLE;
    out.linear_x = 0.0;
    out.remaining_m = 0.0;
    return out;
  }

  const double err_x_world = in.goal_x - in.current_x;
  const double err_x_rail  = err_x_world * in.rail_axis_sign;
  out.remaining_m = std::abs(err_x_rail);

  // Highest-priority gating — collision has overriding stop authority.
  if (in.collision_monitor_stop) {
    out.state = RailState::BLOCKED_WAIT;
    out.linear_x = 0.0;
    return out;
  }

  // Lateral drift abort. Cheap and always-safe to check.
  if (std::abs(in.current_y - in.goal_y) > p.lateral_abort_m) {
    out.state = RailState::BLOCKED_LATERAL;
    out.linear_x = 0.0;
    return out;
  }

  // Yaw abort — only when operating inside a rail aisle (crop-row risk).
  if (in.in_rail_zone && !std::isnan(in.rail_yaw_error) &&
      std::abs(in.rail_yaw_error) > p.yaw_abort_rad) {
    out.state = RailState::BLOCKED_MISALIGNED;
    out.linear_x = 0.0;
    return out;
  }

  // Goal reached (within stop band).
  if (out.remaining_m < p.stop_band_m) {
    out.state = RailState::REACHED;
    out.linear_x = 0.0;
    return out;
  }

  // Nominal P-controller. Sign of err_x_rail may be negative if the robot is
  // past the goal along the rail axis; clamp keeps us inside the speed cap.
  double raw_linear_rail = p.kP * err_x_rail;
  double clamped_rail = std::clamp(raw_linear_rail,
                                   -p.speed_max_mps, p.speed_max_mps);
  // Translate back to robot frame: robot's +x always is forward; when
  // rail_axis_sign == +1 the rail-forward direction equals robot-forward, and
  // when -1 the rail-forward direction is robot-backward. Our command in
  // robot frame is rail-linear * rail_axis_sign.
  out.linear_x = clamped_rail * in.rail_axis_sign;
  out.state = RailState::DRIVING;
  return out;
}

}  // namespace agv_rail_driver
