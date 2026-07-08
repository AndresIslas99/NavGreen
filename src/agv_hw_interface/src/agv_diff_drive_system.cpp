#include "agv_hw_interface/agv_diff_drive_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "agv_hw_interface/odrive_protocol.hpp"
#include "agv_hw_interface/wheel_conversion.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agv_hw_interface {

namespace {
double parse_param(const hardware_interface::HardwareInfo& info,
                   const std::string& key,
                   double fallback) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return fallback;
  try {
    return std::stod(it->second);
  } catch (...) {
    return fallback;
  }
}

int parse_param_int(const hardware_interface::HardwareInfo& info,
                    const std::string& key,
                    int fallback) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return fallback;
  try {
    return std::stoi(it->second);
  } catch (...) {
    return fallback;
  }
}

bool parse_param_bool(const hardware_interface::HardwareInfo& info,
                      const std::string& key,
                      bool fallback) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return fallback;
  return (it->second == "true" || it->second == "True" || it->second == "1");
}

std::string parse_param_str(const hardware_interface::HardwareInfo& info,
                            const std::string& key,
                            const std::string& fallback) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return fallback;
  return it->second;
}
}  // namespace

hardware_interface::CallbackReturn AgvDiffDriveSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.size() != 2) {
    RCLCPP_FATAL(logger_, "AgvDiffDriveSystem expects exactly 2 joints, got %zu",
                 info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto& joint : info.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != "velocity") {
      RCLCPP_FATAL(logger_, "joint %s must have exactly one 'velocity' command interface",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger_, "joint %s must expose 2 state interfaces (position, velocity)",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  can_interface_ = parse_param_str(info, "can_interface", "can0");
  left_axis_id_  = static_cast<uint8_t>(parse_param_int(info, "left_axis_id", 0));
  right_axis_id_ = static_cast<uint8_t>(parse_param_int(info, "right_axis_id", 1));
  gear_ratio_    = parse_param(info, "gear_ratio", 10.0);
  invert_left_   = parse_param_bool(info, "invert_left", true);
  invert_right_  = parse_param_bool(info, "invert_right", false);
  recv_timeout_ms_ = parse_param_int(info, "recv_timeout_ms", 5);
  // Fault thresholds: consecutive-cycle counts at the controller_manager
  // update rate (50 Hz by default) before read()/write() report ERROR.
  max_send_failures_ = std::max(1, parse_param_int(info, "max_send_failures", 10));
  encoder_timeout_cycles_ = std::max(1, parse_param_int(info, "encoder_timeout_cycles", 100));

  if (gear_ratio_ <= 0.0) {
    RCLCPP_FATAL(logger_, "gear_ratio must be > 0 (got %.3f)", gear_ratio_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_,
              "AgvDiffDriveSystem initialized: can=%s left_id=%u right_id=%u "
              "gear=%.2f invert_left=%d invert_right=%d",
              can_interface_.c_str(), left_axis_id_, right_axis_id_,
              gear_ratio_, invert_left_, invert_right_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AgvDiffDriveSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> ifaces;
  // Order matches info_.joints (set by SystemInterface::on_init).
  ifaces.emplace_back(info_.joints[0].name, "position", &left_.position);
  ifaces.emplace_back(info_.joints[0].name, "velocity", &left_.velocity);
  ifaces.emplace_back(info_.joints[1].name, "position", &right_.position);
  ifaces.emplace_back(info_.joints[1].name, "velocity", &right_.velocity);
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
AgvDiffDriveSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.emplace_back(info_.joints[0].name, "velocity", &left_.command);
  ifaces.emplace_back(info_.joints[1].name, "velocity", &right_.command);
  return ifaces;
}

hardware_interface::CallbackReturn AgvDiffDriveSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  can_ = std::make_unique<CANSocket>(can_interface_);
  if (!can_->is_open()) {
    RCLCPP_ERROR(logger_,
                 "failed to open CAN interface %s — staying inactive. "
                 "If you do not have hardware, use the mock launch file instead.",
                 can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Arm both axes (CLOSED_LOOP_CONTROL). A failed send means the bus is
  // already dead — refuse to activate instead of pretending the axes armed.
  for (uint8_t node_id : {left_axis_id_, right_axis_id_}) {
    uint8_t data[8] = {};
    pack_axis_state(data, AxisState::CLOSED_LOOP_CONTROL);
    if (!can_->send(make_arb_id(node_id, cmd::SET_AXIS_STATE), data, 8)) {
      RCLCPP_ERROR(logger_, "failed to send CLOSED_LOOP_CONTROL to axis %u on %s — staying inactive",
                   node_id, can_interface_.c_str());
      can_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Re-latch encoder baselines on every activation — motor positions may have
  // moved (or the ODrive power-cycled) while the plugin was inactive.
  send_fail_count_ = 0;
  stale_read_cycles_ = 0;
  left_.initialized = false;
  right_.initialized = false;

  RCLCPP_INFO(logger_, "AgvDiffDriveSystem activated on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvDiffDriveSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (can_ && can_->is_open()) {
    for (uint8_t node_id : {left_axis_id_, right_axis_id_}) {
      uint8_t data[8] = {};
      pack_velocity(data, 0.0f, 0.0f);
      can_->send(make_arb_id(node_id, cmd::SET_INPUT_VEL), data, 8);
      pack_axis_state(data, AxisState::IDLE);
      can_->send(make_arb_id(node_id, cmd::SET_AXIS_STATE), data, 8);
    }
  }
  can_.reset();
  RCLCPP_INFO(logger_, "AgvDiffDriveSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AgvDiffDriveSystem::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!can_ || !can_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // Request encoder estimates from both axes (RTR)
  request_encoders(left_axis_id_);
  request_encoders(right_axis_id_);

  // Drain any pending CAN frames within the budget. The diff_drive_controller
  // typically runs at 50 Hz so we have ~20 ms; bound the work here so we never
  // starve the rest of the controller_manager loop.
  bool got_encoder_frame = false;
  for (int i = 0; i < 8; ++i) {
    struct can_frame frame{};
    if (!can_->recv(frame, recv_timeout_ms_)) break;

    // Skip RTR polls (our own requests looped back, or another local process
    // polling the same bus) and short frames — they carry no valid payload
    // and would parse as position/velocity 0.0.
    if ((frame.can_id & CAN_RTR_FLAG) != 0 || frame.can_dlc < 8) continue;

    const uint8_t node_id = get_node_id(frame.can_id);
    const uint8_t cmd_id  = get_cmd_id(frame.can_id);
    if (cmd_id == cmd::GET_ENCODER_ESTIMATES) {
      process_encoder_frame(node_id, frame);
      got_encoder_frame = true;
    }
  }

  // Encoder staleness: a SocketCAN fd stays "open" when the interface drops
  // or the ODrive powers off, so frame freshness is the only dead-bus signal.
  if (got_encoder_frame) {
    stale_read_cycles_ = 0;
  } else if (stale_read_cycles_ >= encoder_timeout_cycles_) {
    return hardware_interface::return_type::ERROR;  // reported when first crossed
  } else if (++stale_read_cycles_ >= encoder_timeout_cycles_) {
    RCLCPP_ERROR(logger_,
                 "no encoder frames for %d consecutive read() cycles on %s — "
                 "CAN bus or ODrive unresponsive, reporting ERROR",
                 stale_read_cycles_, can_interface_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvDiffDriveSystem::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!can_ || !can_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  const bool left_ok  = send_velocity(left_axis_id_, left_.command, invert_left_);
  const bool right_ok = send_velocity(right_axis_id_, right_.command, invert_right_);

  // Consecutive send failures are a bus-down signal (ENETDOWN, or tx queue
  // full because no node ACKs) — surface them so controller_manager can
  // trigger its error handling instead of commanding a dead bus forever.
  if (left_ok && right_ok) {
    send_fail_count_ = 0;
  } else if (send_fail_count_ >= max_send_failures_) {
    return hardware_interface::return_type::ERROR;  // reported when first crossed
  } else if (++send_fail_count_ >= max_send_failures_) {
    RCLCPP_ERROR(logger_,
                 "%d consecutive CAN send failures on %s — reporting ERROR",
                 send_fail_count_, can_interface_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool AgvDiffDriveSystem::send_velocity(uint8_t node_id,
                                       double wheel_rad_per_s,
                                       bool invert) {
  const double motor_turns_per_s =
      wheel_rad_to_motor_turns(wheel_rad_per_s, gear_ratio_, invert);

  uint8_t data[8] = {};
  pack_velocity(data, static_cast<float>(motor_turns_per_s), 0.0f);
  return can_->send(make_arb_id(node_id, cmd::SET_INPUT_VEL), data, 8);
}

void AgvDiffDriveSystem::request_encoders(uint8_t node_id) {
  can_->send_rtr(make_arb_id(node_id, cmd::GET_ENCODER_ESTIMATES));
}

void AgvDiffDriveSystem::process_encoder_frame(uint8_t node_id,
                                               const struct can_frame& frame) {
  const auto msg = EncoderMsg::parse(frame.data);
  if (!msg.valid) return;

  WheelState* wheel = nullptr;
  bool invert = false;
  if (node_id == left_axis_id_) {
    wheel = &left_;
    invert = invert_left_;
  } else if (node_id == right_axis_id_) {
    wheel = &right_;
    invert = invert_right_;
  } else {
    return;
  }

  // Convert motor turns to wheel rad
  const double motor_turns = static_cast<double>(msg.position);
  const double motor_vel   = static_cast<double>(msg.velocity);

  if (!wheel->initialized) {
    wheel->last_motor_turns = motor_turns;
    wheel->initialized = true;
    return;
  }

  const double delta_motor_turns = motor_turns - wheel->last_motor_turns;
  wheel->last_motor_turns = motor_turns;

  wheel->position += motor_turns_to_wheel_rad(delta_motor_turns, gear_ratio_, invert);
  wheel->velocity = motor_turns_to_wheel_rad(motor_vel, gear_ratio_, invert);
}

}  // namespace agv_hw_interface

PLUGINLIB_EXPORT_CLASS(agv_hw_interface::AgvDiffDriveSystem,
                       hardware_interface::SystemInterface)
