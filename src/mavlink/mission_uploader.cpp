#include "mavlink/mission_uploader.hpp"

#include <algorithm>
#include <cmath>

#include "mavlink/commands.hpp"  // kGcsSystemId / kGcsComponentId

namespace mgs {
namespace {

// "Keep the current heading" — NaN yaw, as in the COMMAND_LONG builders.
const float kCurrentYaw = NAN;

// Degrees -> the 1e7-scaled integer MISSION_ITEM_INT carries.
std::int32_t to_e7(std::optional<double> deg) {
  return deg ? static_cast<std::int32_t>(std::lround(*deg * 1e7)) : 0;
}

// Pack one mission step as the MISSION_ITEM_INT for slot `seq`.
mavlink_message_t to_message(const MissionItem& item, std::uint16_t seq,
                             std::uint8_t ts, std::uint8_t tc) {
  std::uint16_t command = MAV_CMD_NAV_WAYPOINT;
  float p1 = 0.0f;
  float p4 = kCurrentYaw;
  float z = item.alt_m;
  // Positional steps use the relative-altitude global frame. A step with no
  // position (RTL) must say MAV_FRAME_MISSION, "x/y/z unused": in a global
  // frame PX4 reads x=0 as a latitude and refuses it (INVALID_PARAM5_X).
  std::uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  switch (item.kind) {
    case MissionItemKind::Takeoff:
      command = MAV_CMD_NAV_TAKEOFF;  // p1 = min pitch (0: autopilot default)
      break;
    case MissionItemKind::Waypoint:
      command = MAV_CMD_NAV_WAYPOINT;
      p1 = item.hold_s;  // p2 accept radius, p3 pass radius: 0 = defaults
      break;
    case MissionItemKind::Land:
      command = MAV_CMD_NAV_LAND;
      z = 0.0f;
      break;
    case MissionItemKind::Rtl:
      command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
      frame = MAV_FRAME_MISSION;
      p4 = 0.0f;
      z = 0.0f;
      break;
  }
  mavlink_message_t msg;
  mavlink_msg_mission_item_int_pack(
      kGcsSystemId, kGcsComponentId, &msg, ts, tc, seq,
      frame, command, /*current=*/0,
      /*autocontinue=*/1, p1, /*p2=*/0.0f, /*p3=*/0.0f, p4,
      to_e7(item.lat_deg), to_e7(item.lon_deg), z, MAV_MISSION_TYPE_MISSION);
  return msg;
}

}  // namespace

mavlink_message_t MissionUploader::begin(
    const Mission& mission, std::uint8_t target_system,
    std::uint8_t target_component, std::chrono::steady_clock::time_point now) {
  items_.clear();
  for (std::size_t i = 0; i < mission.size(); ++i) {
    items_.push_back(to_message(mission[i], static_cast<std::uint16_t>(i),
                                target_system, target_component));
  }
  status_ = MissionStatus::Uploading;
  target_system_ = target_system;
  sent_ = 0;
  error_ = 0;

  mavlink_message_t count;
  mavlink_msg_mission_count_pack(kGcsSystemId, kGcsComponentId, &count,
                                 target_system, target_component, this->count(),
                                 MAV_MISSION_TYPE_MISSION, /*opaque_id=*/0);
  return remember(count, now);
}

std::optional<mavlink_message_t> MissionUploader::on_message(
    const mavlink_message_t& msg, std::chrono::steady_clock::time_point now) {
  if (status_ != MissionStatus::Uploading) return std::nullopt;
  if (msg.sysid != target_system_) return std::nullopt;  // another vehicle

  // The vehicle asks for item `seq`. PX4 uses MISSION_REQUEST_INT; the older
  // MISSION_REQUEST asks the same question, and the spec says to answer it with
  // MISSION_ITEM_INT too. Both carry seq + mission_type at the same meaning.
  std::uint16_t seq = 0;
  std::uint8_t type = 0;
  if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
    mavlink_mission_request_int_t r;
    mavlink_msg_mission_request_int_decode(&msg, &r);
    seq = r.seq;
    type = r.mission_type;
  } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
    mavlink_mission_request_t r;
    mavlink_msg_mission_request_decode(&msg, &r);
    seq = r.seq;
    type = r.mission_type;
  } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&msg, &ack);
    if (ack.mission_type != MAV_MISSION_TYPE_MISSION) return std::nullopt;
    if (ack.type == MAV_MISSION_ACCEPTED) {
      status_ = MissionStatus::Uploaded;
    } else {
      status_ = MissionStatus::Rejected;
      error_ = ack.type;
    }
    return std::nullopt;
  } else {
    return std::nullopt;  // not part of the mission protocol
  }

  if (type != MAV_MISSION_TYPE_MISSION || seq >= items_.size()) {
    return std::nullopt;  // a fence/rally request, or a slot we never offered
  }
  sent_ = std::max<std::uint16_t>(sent_, seq + 1);
  return remember(items_[seq], now);
}

std::optional<mavlink_message_t> MissionUploader::retransmit(
    std::chrono::steady_clock::time_point now) {
  if (status_ != MissionStatus::Uploading) return std::nullopt;
  if (now - last_sent_ < kRetryInterval) return std::nullopt;
  if (attempts_ >= kMaxAttempts) {
    status_ = MissionStatus::TimedOut;
    return std::nullopt;
  }
  ++attempts_;
  last_sent_ = now;
  return last_;
}

mavlink_message_t MissionUploader::remember(
    const mavlink_message_t& msg, std::chrono::steady_clock::time_point now) {
  // Every vehicle request is progress, so the retry budget starts over: we
  // only give up when one single step fails kMaxAttempts times in a row.
  last_ = msg;
  last_sent_ = now;
  attempts_ = 1;
  return msg;
}

}  // namespace mgs
