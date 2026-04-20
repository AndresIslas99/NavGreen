// Pure controller logic for the rail driver. Header-only, no ROS deps.
// Enables unit testing of the control law without spinning a node.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
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
  // Visual feedback thresholds — above min_conf AND age below max_age_s, the
  // controller prefers visual lateral/yaw metrics over the pose-based ones.
  double visual_min_conf  = 0.7;
  double visual_max_age_s = 0.5;
};

enum class RailState {
  IDLE,                // No goal
  DRIVING,             // Actively driving toward goal
  REACHED,             // Within stop_band; cmd=0; publishes state then idles
  BLOCKED_WAIT,        // collision_monitor halted us; holding cmd=0
  BLOCKED_MISALIGNED,  // Yaw-from-rail too large to safely move
  BLOCKED_LATERAL,     // Lateral drift exceeded lateral_abort_m
  CANCELED,            // Iter-22: operator canceled via /agv/rail_driver/
                       // cancel_goal service. Distinct from IDLE so
                       // downstream (mode_arbiter, harness) can tell
                       // "never had a goal" from "explicitly canceled".
                       // Latches for one tick; next compute() call with
                       // have_goal=false returns IDLE.
};

inline const char *state_to_str(RailState s) {
  switch (s) {
    case RailState::IDLE:               return "idle";
    case RailState::DRIVING:            return "driving";
    case RailState::REACHED:            return "reached";
    case RailState::BLOCKED_WAIT:       return "blocked_wait";
    case RailState::BLOCKED_MISALIGNED: return "blocked_misaligned";
    case RailState::BLOCKED_LATERAL:    return "blocked_lateral";
    case RailState::CANCELED:           return "canceled";
  }
  return "unknown";
}

struct RailControllerInputs {
  // Pose (map frame)
  double current_x = 0.0;
  double current_y = 0.0;
  // Robot yaw in map frame. Used to project the world-frame error onto the
  // robot body's +X axis so the commanded linear.x is correct regardless of
  // the robot's heading (yaw=0 or yaw=π both work — robot drives body-forward
  // toward goal). Callers that don't track yaw can leave it 0, matching the
  // original "rail assumed aligned with world +X" semantics.
  double current_yaw = 0.0;
  // Goal (map frame).
  double goal_x = 0.0;
  double goal_y = 0.0;
  // Rail axis sign kept for backwards-compat and unit tests, but with
  // current_yaw propagated the controller no longer depends on it for
  // direction; err is computed via body-frame projection.
  double rail_axis_sign = 1.0;
  // From zone_detector: rail_yaw_error is robot heading vs rail axis, NaN if
  // not in a rail aisle. For gap-zone drives, pass yaw directly instead.
  double rail_yaw_error = 0.0;
  // Gating flags
  bool   in_rail_zone = false;           // true → enforce yaw_abort check
  bool   collision_monitor_stop = false; // halt signal from safety chain
  bool   have_goal = false;              // explicit flag — IDLE without a goal

  // ── Visual feedback from agv_rail_detector (Stage K) ─────────────────
  // Lateral offset from the rail-pair midline in base_link Y (signed):
  // positive means the midline is to the robot's left → robot drifted right.
  // |visual_lat_offset| > lateral_abort_m triggers BLOCKED_LATERAL, same as
  // pose-based drift. Used only when visual_confidence > visual_min_conf AND
  // visual_age_s < visual_max_age_s; otherwise the pose-based check applies.
  double visual_lat_offset = 0.0;
  // Rail-axis yaw vs robot +X, radians. Drop-in replacement for
  // rail_yaw_error when visual is trusted.
  double visual_yaw_error = 0.0;
  // Confidence in [0, 1]. 0 means no detection.
  double visual_confidence = 0.0;
  // Seconds since last visual detection. Infinity means never received.
  double visual_age_s = std::numeric_limits<double>::infinity();
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

  // Distance-to-goal in world.
  const double dx_world = in.goal_x - in.current_x;
  const double dy_world = in.goal_y - in.current_y;
  // Body-frame error: positive means goal is in front of the robot, regardless
  // of world-frame yaw. This replaces the old `err_x_rail` sign logic so the
  // controller works with any goal orientation once the robot is pre-aligned
  // (alignment is the mode_arbiter / rail_approach responsibility).
  const double c = std::cos(in.current_yaw);
  const double s = std::sin(in.current_yaw);
  const double err_body_x = dx_world * c + dy_world * s;
  // Keep the old naming for the stop-band comparison and the unit-test
  // API: "remaining_m" is still the along-rail distance magnitude.
  const double err_x_rail = err_body_x;
  out.remaining_m = std::abs(err_x_rail);

  // Highest-priority gating — collision has overriding stop authority.
  if (in.collision_monitor_stop) {
    out.state = RailState::BLOCKED_WAIT;
    out.linear_x = 0.0;
    return out;
  }

  // Visual feedback from rail_detector is preferred when fresh and confident.
  // When rejected (stale / low conf / never received), the pose-based checks
  // below apply instead — same thresholds, same abort states.
  const bool visual_trusted =
      in.visual_confidence > p.visual_min_conf &&
      in.visual_age_s < p.visual_max_age_s;

  // Iter-27 P1 fix: the 51 mm rail tube mechanically constrains lateral
  // drift to < 5 mm while the robot is inside an aisle — the visual
  // lateral offset from rail_detector is reliable during the approach
  // (when the robot has not entered yet and could drift into crop rows)
  // but becomes noisy and over-sensitive as the robot crosses the
  // gap/rail boundary (iter-26f c1/c3_drive_in and iter-27 c1_drive_in
  // all aborted with BLOCKED_LATERAL at x≈3.5, exactly at gap_x_min).
  // Inside a rail aisle we fall back to POSE-based lateral (odometry
  // vs goal_y), which is quieter and still catches genuine drift if
  // the rails ever gave out. Visual yaw-vs-rail-axis check stays
  // active because a yawed robot inside a rail WILL wedge a wheel.
  //
  // Outside rail zone (approach strips / gap transitions): visual
  // lateral is trusted when fresh/confident, since alignment before
  // entry is the whole point of the approach phase.
  if (visual_trusted && !in.in_rail_zone) {
    if (std::abs(in.visual_lat_offset) > p.lateral_abort_m) {
      out.state = RailState::BLOCKED_LATERAL;
      out.linear_x = 0.0;
      return out;
    }
  }

  // Pose-based lateral drift abort — always on. Cheap, quiet, and a
  // genuine safety net even when visual is disabled in-rail.
  if (std::abs(in.current_y - in.goal_y) > p.lateral_abort_m) {
    out.state = RailState::BLOCKED_LATERAL;
    out.linear_x = 0.0;
    return out;
  }

  // Yaw abort inside a rail aisle (crop-row risk). Prefer the visual
  // yaw error (rail-axis vs robot-body) when fresh; fall back to
  // zone_detector's rail_yaw_error otherwise. Off outside rails since
  // the approach flow owns alignment.
  if (in.in_rail_zone) {
    if (visual_trusted && !std::isnan(in.visual_yaw_error) &&
        std::abs(in.visual_yaw_error) > p.yaw_abort_rad) {
      out.state = RailState::BLOCKED_MISALIGNED;
      out.linear_x = 0.0;
      return out;
    }
    if (!visual_trusted && !std::isnan(in.rail_yaw_error) &&
        std::abs(in.rail_yaw_error) > p.yaw_abort_rad) {
      out.state = RailState::BLOCKED_MISALIGNED;
      out.linear_x = 0.0;
      return out;
    }
  }

  // Goal reached (within stop band).
  if (out.remaining_m < p.stop_band_m) {
    out.state = RailState::REACHED;
    out.linear_x = 0.0;
    return out;
  }

  // Nominal P-controller in BODY frame — linear.x is commanded directly in
  // robot's +X. Sign comes from err_body_x (negative if goal is behind the
  // robot; the controller will then command linear.x<0, but with wz=0 the
  // robot can't flip around, so aligned approach is required upstream).
  const double raw_linear = p.kP * err_x_rail;
  out.linear_x = std::clamp(raw_linear, -p.speed_max_mps, p.speed_max_mps);
  out.state = RailState::DRIVING;
  return out;
}

}  // namespace agv_rail_driver
