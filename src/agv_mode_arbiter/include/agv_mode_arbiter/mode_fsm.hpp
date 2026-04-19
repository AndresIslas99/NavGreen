// Pure FSM for the mode arbiter. Header-only, ROS-free.
//
// The arbiter owns /agv/cmd_vel publication. At any instant exactly one of
// three upstream sources is relayed:
//   * cmd_vel_nav      — Nav2 controller_server (corridor zones)
//   * cmd_vel_approach — agv_rail_approach  (AprilTag alignment at rail entry)
//   * cmd_vel_rail     — agv_rail_driver    (longitudinal traversal inside rail)
//
// The operator can force a mode via the `nav | teleop | idle` directive from
// the dashboard (layer 3 of specs/state_machine.yaml). In `teleop` and
// `idle` the arbiter publishes zero velocity and does not relay.
//
// Transitions are driven by observations of the zone, the rail_approach
// service status, the rail_driver state, and safety-chain signals.

#pragma once

#include <cmath>
#include <string>

namespace agv_mode_arbiter {

enum class Mode {
  CORRIDOR_NAV,          // Nav2 owns cmd_vel
  RAIL_APPROACH_PEND,    // Zone entered an approach strip; arbiter called
                         // /agv/rail_approach service, waiting for ACK.
  RAIL_APPROACH_ACTIVE,  // rail_approach is driving (service accepted).
  RAIL_DRIVE,            // rail_approach declared settled; rail_driver
                         // has the goal and is traversing.
  RAIL_EXIT,             // Post-rail transition: rail_driver keeps driving
                         // (wz=0 hard) past the exit AprilTag with a +1 m
                         // extension goal, so Nav2 never rotates inside the
                         // aisle. Source stays RAIL until rail_exit_clearance_m
                         // ≥ 1.0 AND the zone is no longer rail/approach.
  BLOCKED_HANDOFF,       // collision_monitor stop OR e_stop — hold 0 cmd_vel
                         // until the signal clears, then recover to prior mode.
  TELEOP,                // Operator override via /agv/mode/set teleop.
  IDLE,                  // Operator override via /agv/mode/set idle; 0 cmd_vel.
};

enum class Source {
  NONE,         // publish 0 Twist
  NAV,          // relay cmd_vel_nav
  APPROACH,     // relay cmd_vel_approach
  RAIL,         // relay cmd_vel_rail
};

inline const char *mode_to_str(Mode m) {
  switch (m) {
    case Mode::CORRIDOR_NAV:         return "corridor_nav";
    case Mode::RAIL_APPROACH_PEND:   return "rail_approach_pend";
    case Mode::RAIL_APPROACH_ACTIVE: return "rail_approach_active";
    case Mode::RAIL_DRIVE:           return "rail_drive";
    case Mode::RAIL_EXIT:            return "rail_exit";
    case Mode::BLOCKED_HANDOFF:      return "blocked_handoff";
    case Mode::TELEOP:               return "teleop";
    case Mode::IDLE:                 return "idle";
  }
  return "unknown";
}

inline const char *source_to_str(Source s) {
  switch (s) {
    case Source::NONE:     return "none";
    case Source::NAV:      return "nav";
    case Source::APPROACH: return "approach";
    case Source::RAIL:     return "rail";
  }
  return "unknown";
}

struct FsmInputs {
  // Operator directive: "nav" | "teleop" | "idle".
  std::string operator_mode = "nav";
  // If true, the arbiter auto-fires /agv/rail_approach/execute whenever the
  // robot enters an approach strip. Default false: the arbiter is an
  // observer/router, not an orchestrator. Tests and the dashboard retain
  // explicit control over when rail_approach runs (by calling the service
  // themselves). Enable this flag in operational profiles that want the
  // hands-off docking behaviour.
  bool auto_approach = false;
  // Zone label from /agv/zone/state.
  std::string zone = "gap";
  // Floor AprilTag ID at the current approach strip (-1 if not an approach).
  // Read from /agv/zone/state's approach_tag_id field by the ROS wrapper.
  int approach_tag_id = -1;
  // Aisle center y from /agv/zone/state (used to derive rail_drive goals).
  // NaN if zone is not rail-related.
  double aisle_y_center = std::nan("");
  // Current robot pose in map frame — used to derive rail_drive goals.
  // NaN sentinels mean "pose unknown, hold".
  double current_x = std::nan("");
  double current_y = std::nan("");
  // /agv/rail_approach/state.state ("idle" | "driving" | "settled" | "aborted").
  std::string rail_approach_state = "idle";
  // /agv/rail_driver/state.state (see agv_rail_driver/rail_controller.hpp).
  std::string rail_driver_state = "idle";
  // True when collision_monitor published "stop" (or "e_stop").
  bool safety_stop = false;
  // True when the arbiter has already dispatched the rail_approach service
  // call for the current approach window. Consumed so the arbiter does not
  // double-call on every FSM tick.
  bool approach_request_in_flight = false;
  // Signed clearance past the last exit AprilTag, in metres along the rail's
  // axial direction. ≥ 1.0 means "robot is at least 1 m past the tag and
  // safe to rotate"; below that, the arbiter must keep source=RAIL so
  // rail_driver's wz=0 hard lock prevents Nav2 from turning near crop rows.
  // Default 0 so the FSM errs on the side of holding RAIL_EXIT.
  double rail_exit_clearance_m = 0.0;
};

struct FsmOutputs {
  Mode next_mode = Mode::CORRIDOR_NAV;
  Source active_source = Source::NAV;
  // True if the arbiter should call /agv/rail_approach service NOW
  // (transition into RAIL_APPROACH_PEND).
  bool request_rail_approach = false;
  // True if the arbiter should publish /agv/rail_driver/goal to hand off
  // longitudinal traversal to the rail driver (transition into RAIL_DRIVE).
  bool request_rail_drive_goal = false;
  // True when RAIL_DRIVE just latched "reached" and the arbiter must now
  // publish an extended goal 1 m past the exit tag so the robot clears the
  // aisle before Nav2 is allowed to rotate. Consumed once per transition.
  bool request_rail_exit_push = false;
};

inline bool is_approach_zone(const std::string &zone) {
  return zone == "rail_approach_front" || zone == "rail_approach_rear";
}

inline bool is_rail_zone(const std::string &zone) {
  return zone == "rail_aisle_0"   || zone == "rail_aisle_p22" ||
         zone == "rail_aisle_m22" || zone == "rail_aisle_p44" ||
         zone == "rail_aisle_m44";
}

inline FsmOutputs step(Mode current, const FsmInputs &in) {
  FsmOutputs out;
  out.next_mode = current;

  // Operator overrides — top priority.
  if (in.operator_mode == "idle") {
    out.next_mode = Mode::IDLE;
    out.active_source = Source::NONE;
    return out;
  }
  if (in.operator_mode == "teleop") {
    out.next_mode = Mode::TELEOP;
    out.active_source = Source::NONE;  // teleop_server owns /agv/cmd_vel directly
    return out;
  }

  // Safety chain — second priority. On release, fall back to CORRIDOR_NAV
  // and let the zone observer re-enter the appropriate rail flow.
  if (in.safety_stop) {
    out.next_mode = Mode::BLOCKED_HANDOFF;
    out.active_source = Source::NONE;
    return out;
  }

  // FSM transitions proper.
  switch (current) {
    case Mode::CORRIDOR_NAV:
      // Direct-dispatch shortcut: if something external published a goal
      // straight to /agv/rail_driver/goal, rail_driver's state reports
      // "driving". Swap source to RAIL so cmd_vel_rail is relayed.
      // Supports test harnesses (Stage G dispatch) and operator consoles
      // that bypass the auto-approach ceremony.
      if (in.rail_driver_state == "driving") {
        out.next_mode = Mode::RAIL_DRIVE;
        out.active_source = Source::RAIL;
        return out;
      }
      // Similarly: if rail_approach was fired externally and already reached
      // fine_servoing ("driving" bucket), we should have been observing. This
      // covers the case where the arbiter booted mid-approach.
      if (in.rail_approach_state == "driving") {
        out.next_mode = Mode::RAIL_APPROACH_ACTIVE;
        out.active_source = Source::APPROACH;
        return out;
      }
      // Auto-trigger only when explicitly opted in. Without opt-in the
      // arbiter stays in CORRIDOR_NAV and source=NAV even inside approach
      // strips — the caller (test harness / operator UI) is responsible
      // for firing /agv/rail_approach/execute when needed.
      if (in.auto_approach && is_approach_zone(in.zone) &&
          !in.approach_request_in_flight) {
        out.next_mode = Mode::RAIL_APPROACH_PEND;
        out.active_source = Source::NAV;
        out.request_rail_approach = true;
        return out;
      }
      // Even if rail_approach is running as a result of an explicit service
      // call, catch its "driving" transition here so the arbiter still
      // swaps cmd_vel to the approach source at fine-servoing.
      if (in.rail_approach_state == "driving") {
        out.next_mode = Mode::RAIL_APPROACH_ACTIVE;
        out.active_source = Source::APPROACH;
        return out;
      }
      out.active_source = Source::NAV;
      return out;

    case Mode::RAIL_APPROACH_PEND:
      // Wait for rail_approach to enter FINE_SERVOING (= "driving"). During
      // the preceding COARSE_APPROACH / TAG_ACQUISITION sub-states the
      // Nav2 stack is the active cmd_vel publisher on rail_approach's
      // behalf — keep source=NAV until rail_approach takes over its own
      // cmd_vel.
      if (in.rail_approach_state == "driving") {
        out.next_mode = Mode::RAIL_APPROACH_ACTIVE;
        out.active_source = Source::APPROACH;
        return out;
      }
      if (in.rail_approach_state == "aborted") {
        out.next_mode = Mode::CORRIDOR_NAV;
        out.active_source = Source::NAV;
        return out;
      }
      out.active_source = Source::NAV;
      return out;

    case Mode::RAIL_APPROACH_ACTIVE:
      if (in.rail_approach_state == "settled") {
        out.next_mode = Mode::RAIL_DRIVE;
        out.active_source = Source::RAIL;
        out.request_rail_drive_goal = true;
        return out;
      }
      if (in.rail_approach_state == "aborted") {
        out.next_mode = Mode::CORRIDOR_NAV;
        out.active_source = Source::NAV;
        return out;
      }
      out.active_source = Source::APPROACH;
      return out;

    case Mode::RAIL_DRIVE:
      // Goal reached → enter RAIL_EXIT. Arbiter publishes an extended goal
      // 1 m past the exit tag so the robot keeps driving with wz=0 until
      // fully clear of the aisle. Never hand directly to Nav2 here: Nav2's
      // MPPI would sample rotations and clip the 51 mm rail tubes.
      if (in.rail_driver_state == "reached") {
        out.next_mode = Mode::RAIL_EXIT;
        out.active_source = Source::RAIL;
        out.request_rail_exit_push = true;
        return out;
      }
      // Aborts inside a rail aisle: stay on RAIL source so rail_driver (wz=0)
      // keeps authority. The operator can issue a reverse goal to back out
      // through /agv/rail_driver/goal; Nav2 must not take over inside the
      // aisle because it can rotate into crop rows.
      if (in.rail_driver_state == "blocked_lateral" ||
          in.rail_driver_state == "blocked_misaligned") {
        out.next_mode = Mode::RAIL_EXIT;
        out.active_source = Source::RAIL;
        return out;
      }
      out.active_source = Source::RAIL;
      return out;

    case Mode::RAIL_EXIT:
      // Stay on rail_driver until (a) the robot has cleared the approach +
      // rail zones AND (b) rail_exit_clearance_m ≥ 1 m past the exit tag.
      // Only then can Nav2 safely resume — rotation is legal from here on.
      if (in.rail_driver_state == "reached" &&
          !is_rail_zone(in.zone) && !is_approach_zone(in.zone) &&
          in.rail_exit_clearance_m >= 1.0) {
        out.next_mode = Mode::CORRIDOR_NAV;
        out.active_source = Source::NAV;
        return out;
      }
      out.active_source = Source::RAIL;
      return out;

    case Mode::BLOCKED_HANDOFF:
      // Released from safety_stop; fall back to corridor. The zone observer
      // will re-enter an approach window on the next tick if we happen to be
      // sitting in an approach strip.
      out.next_mode = Mode::CORRIDOR_NAV;
      out.active_source = Source::NAV;
      return out;

    case Mode::TELEOP:
    case Mode::IDLE:
      // Unreachable given the operator-override branch above; kept for
      // exhaustiveness.
      out.next_mode = Mode::CORRIDOR_NAV;
      out.active_source = Source::NAV;
      return out;
  }
  return out;
}

}  // namespace agv_mode_arbiter
