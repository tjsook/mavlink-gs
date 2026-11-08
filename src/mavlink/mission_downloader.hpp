#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include <common/mavlink.h>

#include "state/vehicle_state.hpp"  // DownloadStatus, VehicleMissionItem

namespace mgs {

// Drives one mission download through the MAVLink mission protocol.
//
// The mirror image of MissionUploader: now *we* are the side that pulls. We ask
// for the list (MISSION_REQUEST_LIST); the vehicle says how many items it holds
// (MISSION_COUNT); we ask for each by number (MISSION_REQUEST_INT) and it sends
// that item (MISSION_ITEM_INT); after the last one we send MISSION_ACK so the
// vehicle knows the transfer is over. A lost item is just asked for again.
//
// Deterministic like the uploader: `now` is injected and the caller does the
// sends. Each call that may need a send returns the message, or nullopt.
class MissionDownloader {
 public:
  // Same timing as the uploader: the MAVLink mission-protocol default timeout,
  // and how many sends of one message we try before giving up.
  static constexpr auto kRetryInterval = std::chrono::milliseconds(1500);
  static constexpr int kMaxAttempts = 5;

  // Start fetching the vehicle's mission. Returns the MISSION_REQUEST_LIST to
  // send. Replaces any download in progress.
  mavlink_message_t begin(std::uint8_t target_system,
                          std::uint8_t target_component,
                          std::chrono::steady_clock::time_point now);

  // Feed every received message. Returns our next request, or the final ack,
  // if this message was the count or the item we were waiting for. Also
  // repeats the final ack if the vehicle resends the last item (our ack was
  // lost, so it is still waiting for one).
  std::optional<mavlink_message_t> on_message(
      const mavlink_message_t& msg, std::chrono::steady_clock::time_point now);

  // Call once per loop. Returns our last message again if the vehicle has
  // been silent for kRetryInterval; gives up (TimedOut) after kMaxAttempts.
  std::optional<mavlink_message_t> retransmit(
      std::chrono::steady_clock::time_point now);

  DownloadStatus status() const { return status_; }
  std::uint16_t count() const { return count_; }
  std::uint16_t received() const {
    return static_cast<std::uint16_t>(items_.size());
  }
  // The mission so far; the whole mission once status() is Downloaded.
  const std::vector<VehicleMissionItem>& items() const { return items_; }

 private:
  // Ask for item `seq`, or send the closing ack if we hold them all.
  mavlink_message_t next(std::chrono::steady_clock::time_point now);
  // Remember `msg` as the thing to resend, and restart the silence timer.
  mavlink_message_t remember(const mavlink_message_t& msg,
                             std::chrono::steady_clock::time_point now);

  DownloadStatus status_ = DownloadStatus::None;
  std::uint8_t target_system_ = 0;
  std::uint8_t target_component_ = 0;
  bool counted_ = false;  // MISSION_COUNT has arrived
  std::uint16_t count_ = 0;
  std::vector<VehicleMissionItem> items_;
  mavlink_message_t last_{};  // what retransmit() resends
  std::chrono::steady_clock::time_point last_sent_;
  int attempts_ = 0;
};

// Decode one MISSION_ITEM_INT into what the dashboard (and map) draw. Gives a
// position only to a step that flies somewhere and is in a global frame.
VehicleMissionItem from_mission_item(const mavlink_mission_item_int_t& item);

}  // namespace mgs
