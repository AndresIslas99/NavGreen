#pragma once

// Minimal SocketCAN wrapper. Mirrors agv_odrive::CANSocket. Duplicated here so
// agv_hw_interface can be built without a build-time dependency on agv_odrive.
// When the migration in docs/architectural_gaps.md (Gap 1) completes and
// agv_odrive is removed, this is the only copy that survives.

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

namespace agv_hw_interface {

class CANSocket {
 public:
  explicit CANSocket(const std::string& interface);
  ~CANSocket();

  CANSocket(const CANSocket&) = delete;
  CANSocket& operator=(const CANSocket&) = delete;

  bool is_open() const;
  bool send(uint32_t arb_id, const uint8_t* data, uint8_t dlc);
  bool send_rtr(uint32_t arb_id);
  bool recv(struct can_frame& frame, int timeout_ms);
  void close();

 private:
  int fd_ = -1;
  std::string interface_;
};

}  // namespace agv_hw_interface
