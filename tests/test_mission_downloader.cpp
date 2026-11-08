#include <chrono>

#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "mavlink/commands.hpp"
#include "mavlink/mission_downloader.hpp"

namespace {

using namespace std::chrono_literals;
using mgs::DownloadStatus;
const auto t0 = std::chrono::steady_clock::time_point{} + 100s;

constexpr std::uint8_t kVehicle = 1;

// What the vehicle sends first: "I hold `n` items".
mavlink_message_t count(std::uint16_t n, std::uint8_t from = kVehicle) {
  mavlink_message_t m;
  mavlink_msg_mission_count_pack(from, 1, &m, mgs::kGcsSystemId,
                                 mgs::kGcsComponentId, n,
                                 MAV_MISSION_TYPE_MISSION, 0);
  return m;
}

// What the vehicle sends for each request: item `seq`.
mavlink_message_t item(std::uint16_t seq,
                       std::uint16_t command = MAV_CMD_NAV_WAYPOINT,
                       std::uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       std::int32_t x = 473980000, std::int32_t y = 85460000,
                       float z = 20.0f) {
  mavlink_message_t m;
  mavlink_msg_mission_item_int_pack(kVehicle, 1, &m, mgs::kGcsSystemId,
                                    mgs::kGcsComponentId, seq, frame, command,
                                    0, 1, 0, 0, 0, 0, x, y, z,
                                    MAV_MISSION_TYPE_MISSION);
  return m;
}

// The seq a MISSION_REQUEST_INT of ours asks for.
std::uint16_t requested(const mavlink_message_t& m) {
  REQUIRE(m.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT);
  mavlink_mission_request_int_t r;
  mavlink_msg_mission_request_int_decode(&m, &r);
  CHECK(r.target_system == kVehicle);
  return r.seq;
}

bool is_accepting_ack(const mavlink_message_t& m) {
  if (m.msgid != MAVLINK_MSG_ID_MISSION_ACK) return false;
  mavlink_mission_ack_t ack;
  mavlink_msg_mission_ack_decode(&m, &ack);
  return ack.type == MAV_MISSION_ACCEPTED;
}

}  // namespace

TEST_CASE("begin asks for the mission list", "[download]") {
  mgs::MissionDownloader down;
  const auto list = down.begin(kVehicle, 1, t0);
  CHECK(list.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST);
  CHECK(down.status() == DownloadStatus::Downloading);
}

TEST_CASE("a full download: count, each item in turn, then our ack",
          "[download]") {
  mgs::MissionDownloader down;
  down.begin(kVehicle, 1, t0);

  CHECK(requested(*down.on_message(count(3), t0 + 10ms)) == 0);
  CHECK(requested(*down.on_message(item(0, MAV_CMD_NAV_TAKEOFF), t0)) == 1);
  CHECK(requested(*down.on_message(item(1), t0)) == 2);
  CHECK(down.received() == 2);

  const auto ack = down.on_message(
      item(2, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_FRAME_MISSION, 0, 0, 0), t0);
  REQUIRE(ack);
  CHECK(is_accepting_ack(*ack));
  CHECK(down.status() == DownloadStatus::Downloaded);

  const auto& items = down.items();
  REQUIRE(items.size() == 3);
  CHECK(items[0].command == MAV_CMD_NAV_TAKEOFF);
  CHECK(items[1].lat_deg == 47.398);
  CHECK(items[1].lon_deg == 8.546);
  CHECK(items[1].alt_m == 20.0f);
  CHECK_FALSE(items[2].lat_deg);  // RTL flies home, not to a place
}

TEST_CASE("an empty mission is acked at once", "[download]") {
  mgs::MissionDownloader down;
  down.begin(kVehicle, 1, t0);
  const auto ack = down.on_message(count(0), t0);
  REQUIRE(ack);
  CHECK(is_accepting_ack(*ack));
  CHECK(down.status() == DownloadStatus::Downloaded);
  CHECK(down.items().empty());
}

TEST_CASE("duplicates and strangers are ignored", "[download]") {
  mgs::MissionDownloader down;
  down.begin(kVehicle, 1, t0);
  CHECK_FALSE(down.on_message(item(0), t0));  // an item before the count
  CHECK_FALSE(down.on_message(count(3, /*from=*/7), t0));  // other vehicle
  down.on_message(count(3), t0);
  CHECK_FALSE(down.on_message(count(3), t0));  // count again: already asked
  down.on_message(item(0), t0);
  CHECK_FALSE(down.on_message(item(0), t0));  // item 0 again
  CHECK_FALSE(down.on_message(item(2), t0));  // skipped ahead
  CHECK(down.received() == 1);
}

TEST_CASE("the last item resent after our ack gets the ack again",
          "[download]") {
  mgs::MissionDownloader down;
  down.begin(kVehicle, 1, t0);
  down.on_message(count(1), t0);
  REQUIRE(down.on_message(item(0), t0));  // the ack
  const auto again = down.on_message(item(0), t0 + 2s);
  REQUIRE(again);
  CHECK(is_accepting_ack(*again));
  CHECK(down.items().size() == 1);  // not stored twice
}

TEST_CASE("silence resends the last request, then times out", "[download]") {
  mgs::MissionDownloader down;
  down.begin(kVehicle, 1, t0);
  down.on_message(count(2), t0);  // attempt 1: request item 0

  CHECK_FALSE(down.retransmit(t0 + 1s));  // not silent long enough yet
  auto t = t0;
  for (int attempt = 2; attempt <= mgs::MissionDownloader::kMaxAttempts;
       ++attempt) {
    t += mgs::MissionDownloader::kRetryInterval;
    const auto again = down.retransmit(t);
    REQUIRE(again);
    CHECK(requested(*again) == 0);
  }
  t += mgs::MissionDownloader::kRetryInterval;
  CHECK_FALSE(down.retransmit(t));  // budget spent
  CHECK(down.status() == DownloadStatus::TimedOut);
}

TEST_CASE("from_mission_item gives positions only to steps that fly there",
          "[download]") {
  mavlink_mission_item_int_t raw{};
  raw.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  raw.x = 473980000;
  raw.y = 85460000;

  SECTION("a waypoint has one") {
    raw.command = MAV_CMD_NAV_WAYPOINT;
    CHECK(mgs::from_mission_item(raw).lat_deg);
  }
  SECTION("a speed change does not, even with numbers in x/y") {
    raw.command = MAV_CMD_DO_CHANGE_SPEED;
    CHECK_FALSE(mgs::from_mission_item(raw).lat_deg);
  }
  SECTION("a local frame does not") {
    raw.command = MAV_CMD_NAV_WAYPOINT;
    raw.frame = MAV_FRAME_LOCAL_NED;
    CHECK_FALSE(mgs::from_mission_item(raw).lat_deg);
  }
  SECTION("0,0 means 'here', not the Gulf of Guinea") {
    raw.command = MAV_CMD_NAV_LAND;
    raw.x = raw.y = 0;
    CHECK_FALSE(mgs::from_mission_item(raw).lat_deg);
  }
}
