#pragma once

#include <chrono>
#include <cstdint>

#include <common/mavlink.h>

#include "state/vehicle_state.hpp"  // CommandStatus

namespace mgs {

// Drives one in-flight command's send / retransmit / acknowledge lifecycle.
//
// MAVLink commands ride on UDP and can be lost, so a ground station resends
// until the vehicle acknowledges or a retry budget runs out. This is the
// minimal form of that protocol. It is deterministic: the caller injects `now`
// and does the actual sends, so it is fully unit-testable without a socket or
// real time. Only one command is tracked at a time (newest replaces any prior).
class CommandTracker {
 public:
  static constexpr auto kRetryInterval = std::chrono::seconds(1);
  static constexpr int kMaxAttempts = 3;

  // Start tracking a freshly-sent command. The caller sends it once first, then
  // calls begin() to count that as attempt 1. Replaces any command in flight.
  void begin(std::uint16_t command, std::chrono::steady_clock::time_point now);

  // Feed a received COMMAND_ACK. If it matches the in-flight command, record the
  // outcome and stop retransmitting (any ack means the vehicle heard us).
  void on_ack(const mavlink_command_ack_t& ack);

  // Call once per loop. Returns true if a retransmit is due — the caller should
  // resend the same message. Gives up after kMaxAttempts, moving to TimedOut.
  bool should_resend(std::chrono::steady_clock::time_point now);

  CommandStatus status() const { return status_; }
  std::uint16_t command() const { return command_; }

 private:
  CommandStatus status_ = CommandStatus::None;
  std::uint16_t command_ = 0;
  int attempts_ = 0;
  bool acknowledged_ = false;  // any matching ack seen -> stop resending
  std::chrono::steady_clock::time_point last_sent_;
};

}  // namespace mgs
