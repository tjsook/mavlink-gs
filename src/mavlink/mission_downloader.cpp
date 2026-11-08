#include "mavlink/mission_downloader.hpp"

#include "mavlink/commands.hpp"  // kGcsSystemId / kGcsComponentId

namespace mgs {
namespace {

// Frames whose x/y are a latitude/longitude (scaled 1e7 in MISSION_ITEM_INT).
bool is_global(std::uint8_t frame) {
  switch (frame) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
    case MAV_FRAME_GLOBAL_INT:
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
      return true;
    default:
      return false;
  }
}

}  // namespace

VehicleMissionItem from_mission_item(const mavlink_mission_item_int_t& item) {
  VehicleMissionItem out;
  out.command = item.command;
  out.frame = item.frame;
  out.alt_m = item.z;
  // NAV commands (below MAV_CMD_NAV_LAST) move the vehicle; the rest (speed,
  // camera, ...) do not, even if they carry numbers in x/y. RTL flies home,
  // not to its x/y. A 0,0 position means "here" (e.g. land where you are).
  const bool flies_to_xy = item.command < MAV_CMD_NAV_LAST &&
                           item.command != MAV_CMD_NAV_RETURN_TO_LAUNCH;
  if (flies_to_xy && is_global(item.frame) && (item.x != 0 || item.y != 0)) {
    out.lat_deg = item.x / 1e7;
    out.lon_deg = item.y / 1e7;
  }
  return out;
}

mavlink_message_t MissionDownloader::begin(
    std::uint8_t target_system, std::uint8_t target_component,
    std::chrono::steady_clock::time_point now) {
  status_ = DownloadStatus::Downloading;
  target_system_ = target_system;
  target_component_ = target_component;
  counted_ = false;
  count_ = 0;
  items_.clear();

  mavlink_message_t list;
  mavlink_msg_mission_request_list_pack(kGcsSystemId, kGcsComponentId, &list,
                                        target_system, target_component,
                                        MAV_MISSION_TYPE_MISSION);
  return remember(list, now);
}

std::optional<mavlink_message_t> MissionDownloader::on_message(
    const mavlink_message_t& msg, std::chrono::steady_clock::time_point now) {
  if (msg.sysid != target_system_) return std::nullopt;  // another vehicle

  if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
    if (status_ != DownloadStatus::Downloading) return std::nullopt;
    mavlink_mission_count_t c;
    mavlink_msg_mission_count_decode(&msg, &c);
    // A second count is the answer to a resent list request; the item request
    // we already sent (and retransmit()) covers it.
    if (c.mission_type != MAV_MISSION_TYPE_MISSION || counted_) {
      return std::nullopt;
    }
    counted_ = true;
    count_ = c.count;
    items_.reserve(count_);
    return next(now);
  }

  if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);
    if (item.mission_type != MAV_MISSION_TYPE_MISSION) return std::nullopt;
    // The last item again after we finished: our ack was lost. Ack again, or
    // the vehicle reports the transfer as failed.
    if (status_ == DownloadStatus::Downloaded && count_ > 0 &&
        item.seq == count_ - 1) {
      return next(now);
    }
    // Only the item we asked for counts. Anything else is a duplicate of one
    // we already hold, or an answer to a request we have since moved past.
    if (status_ != DownloadStatus::Downloading || !counted_ ||
        item.seq != items_.size()) {
      return std::nullopt;
    }
    items_.push_back(from_mission_item(item));
    return next(now);
  }

  return std::nullopt;  // not part of the download
}

std::optional<mavlink_message_t> MissionDownloader::retransmit(
    std::chrono::steady_clock::time_point now) {
  if (status_ != DownloadStatus::Downloading) return std::nullopt;
  if (now - last_sent_ < kRetryInterval) return std::nullopt;
  if (attempts_ >= kMaxAttempts) {
    status_ = DownloadStatus::TimedOut;
    return std::nullopt;
  }
  ++attempts_;
  last_sent_ = now;
  return last_;
}

mavlink_message_t MissionDownloader::next(
    std::chrono::steady_clock::time_point now) {
  mavlink_message_t msg;
  if (items_.size() < count_) {
    mavlink_msg_mission_request_int_pack(
        kGcsSystemId, kGcsComponentId, &msg, target_system_, target_component_,
        static_cast<std::uint16_t>(items_.size()), MAV_MISSION_TYPE_MISSION);
    return remember(msg, now);
  }
  // We hold every item (or there were none): close the transfer. Nothing to
  // wait for after this, so it is not remembered for retransmit().
  status_ = DownloadStatus::Downloaded;
  mavlink_msg_mission_ack_pack(kGcsSystemId, kGcsComponentId, &msg,
                               target_system_, target_component_,
                               MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION,
                               /*opaque_id=*/0);
  return msg;
}

mavlink_message_t MissionDownloader::remember(
    const mavlink_message_t& msg, std::chrono::steady_clock::time_point now) {
  // Each answer from the vehicle is progress, so the retry budget starts over.
  last_ = msg;
  last_sent_ = now;
  attempts_ = 1;
  return msg;
}

}  // namespace mgs
