#pragma once

// Mirrors the subset of agv_odrive's odrive_protocol.hpp that this plugin
// needs. Duplicated for the same reason as can_socket.hpp.

#include <cstdint>
#include <cstring>
#include <cmath>

namespace agv_hw_interface {

inline uint32_t make_arb_id(uint8_t node_id, uint8_t cmd_id) {
  return (static_cast<uint32_t>(node_id) << 5) | cmd_id;
}

inline uint8_t get_node_id(uint32_t arb_id) {
  return static_cast<uint8_t>(arb_id >> 5);
}

inline uint8_t get_cmd_id(uint32_t arb_id) {
  return static_cast<uint8_t>(arb_id & 0x1F);
}

namespace cmd {
constexpr uint8_t HEARTBEAT             = 0x01;
constexpr uint8_t SET_AXIS_STATE        = 0x07;
constexpr uint8_t GET_ENCODER_ESTIMATES = 0x09;
constexpr uint8_t SET_INPUT_VEL         = 0x0D;
}  // namespace cmd

enum class AxisState : uint8_t {
  IDLE                = 1,
  CLOSED_LOOP_CONTROL = 8,
};

struct EncoderMsg {
  float position;  // turns
  float velocity;  // turns/s
  bool valid;

  static EncoderMsg parse(const uint8_t* data) {
    EncoderMsg msg{};
    std::memcpy(&msg.position, &data[0], 4);
    std::memcpy(&msg.velocity, &data[4], 4);
    msg.valid = std::isfinite(msg.position) && std::isfinite(msg.velocity);
    return msg;
  }
};

inline void pack_velocity(uint8_t* data, float velocity, float torque_ff = 0.0f) {
  std::memcpy(&data[0], &velocity, 4);
  std::memcpy(&data[4], &torque_ff, 4);
}

inline void pack_axis_state(uint8_t* data, AxisState state) {
  uint32_t s = static_cast<uint32_t>(state);
  std::memcpy(&data[0], &s, 4);
  std::memset(&data[4], 0, 4);
}

}  // namespace agv_hw_interface
