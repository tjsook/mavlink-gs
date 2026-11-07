#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include <common/mavlink.h>

#include "mission/mission.hpp"
#include "state/vehicle_state.hpp"  // MissionStatus

namespace mgs {

// Drives one mission upload through the MAVLink mission protocol.
//
// The vehicle drives the transfer, not us. We announce how many items there
// are (MISSION_COUNT); the vehicle then asks for each by number
// (MISSION_REQUEST_INT) and we answer with that item (MISSION_ITEM_INT); when it
// has them all it replies MISSION_ACK. Asking by number makes loss recovery
// simple: a lost item is just asked for again.
//
// Like CommandTracker it is deterministic — `now` is injected and the caller
// does the sends — so the whole exchange is unit-testable with fake messages.
// Each call that may need a send returns the message to send, or nullopt.
class MissionUploader {
 public:
  // How long the vehicle may stay silent before we resend our last message,
  // and how many sends of one message we try before giving up. 1.5 s matches
  // the MAVLink mission-protocol default timeout.
  static constexpr auto kRetryInterval = std::chrono::milliseconds(1500);
  static constexpr int kMaxAttempts = 5;

  // Start uploading `mission` (positions already resolved) to the vehicle.
  // Returns the MISSION_COUNT to send. Replaces any upload in progress.
  mavlink_message_t begin(const Mission& mission, std::uint8_t target_system,
                          std::uint8_t target_component,
                          std::chrono::steady_clock::time_point now);

  // Feed every received message. Returns the item the vehicle asked for, if
  // this message was such a request; records the outcome if it was the ack.
  std::optional<mavlink_message_t> on_message(
      const mavlink_message_t& msg, std::chrono::steady_clock::time_point now);

  // Call once per loop. Returns our last message again if the vehicle has
  // been silent for kRetryInterval; gives up (TimedOut) after kMaxAttempts.
  std::optional<mavlink_message_t> retransmit(
      std::chrono::steady_clock::time_point now);

  MissionStatus status() const { return status_; }
  std::uint16_t count() const { return static_cast<std::uint16_t>(items_.size()); }
  std::uint16_t sent() const { return sent_; }
  std::uint8_t error() const { return error_; }  // MAV_MISSION_RESULT

 private:
  // Remember `msg` as the thing to resend, and restart the silence timer.
  mavlink_message_t remember(const mavlink_message_t& msg,
                             std::chrono::steady_clock::time_point now);

  MissionStatus status_ = MissionStatus::None;
  std::vector<mavlink_message_t> items_;  // pre-built MISSION_ITEM_INTs
  std::uint8_t target_system_ = 0;
  std::uint16_t sent_ = 0;    // highest item number requested, plus one
  std::uint8_t error_ = 0;
  mavlink_message_t last_{};  // what retransmit() resends
  std::chrono::steady_clock::time_point last_sent_;
  int attempts_ = 0;
};

}  // namespace mgs
