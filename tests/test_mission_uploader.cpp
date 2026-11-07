#include <chrono>

#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "mavlink/commands.hpp"
#include "mavlink/mission_uploader.hpp"
#include "mission/mission.hpp"

namespace {

using namespace std::chrono_literals;
using mgs::MissionStatus;
const auto t0 = std::chrono::steady_clock::time_point{} + 100s;

constexpr std::uint8_t kVehicle = 1;

// A three-step mission with every position filled in.
mgs::Mission three_items() {
  return mgs::resolve_positions(mgs::parse_mission(R"(
items:
  - { type: takeoff, alt: 10 }
  - { type: waypoint, lat: 47.3980, lon: 8.5460, alt: 20, hold: 2 }
  - { type: rtl }
)"),
                                47.0, 8.0);
}

// What the vehicle sends: "give me item `seq`".
mavlink_message_t request(std::uint16_t seq, std::uint8_t from = kVehicle) {
  mavlink_message_t m;
  mavlink_msg_mission_request_int_pack(from, 1, &m, mgs::kGcsSystemId,
                                       mgs::kGcsComponentId, seq,
                                       MAV_MISSION_TYPE_MISSION);
  return m;
}

// What the vehicle sends at the end: accepted, or a reason it refused.
mavlink_message_t mission_ack(std::uint8_t result) {
  mavlink_message_t m;
  mavlink_msg_mission_ack_pack(kVehicle, 1, &m, mgs::kGcsSystemId,
                               mgs::kGcsComponentId, result,
                               MAV_MISSION_TYPE_MISSION, 0);
  return m;
}

mavlink_mission_item_int_t as_item(const mavlink_message_t& m) {
  REQUIRE(m.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT);
  mavlink_mission_item_int_t item;
  mavlink_msg_mission_item_int_decode(&m, &item);
  return item;
}

}  // namespace

TEST_CASE("begin announces the item count", "[mission]") {
  mgs::MissionUploader up;
  const auto count = up.begin(three_items(), kVehicle, 1, t0);

  REQUIRE(count.msgid == MAVLINK_MSG_ID_MISSION_COUNT);
  mavlink_mission_count_t c;
  mavlink_msg_mission_count_decode(&count, &c);
  CHECK(c.count == 3);
  CHECK(c.target_system == kVehicle);
  CHECK(up.status() == MissionStatus::Uploading);
  CHECK(up.sent() == 0);
}

TEST_CASE("a full upload: requests answered in order, then accepted",
          "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);

  const auto takeoff = as_item(*up.on_message(request(0), t0 + 10ms));
  CHECK(takeoff.seq == 0);
  CHECK(takeoff.command == MAV_CMD_NAV_TAKEOFF);
  CHECK(takeoff.x == 470000000);  // filled from the vehicle's position
  CHECK(takeoff.z == 10.0f);
  CHECK(takeoff.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);

  const auto wp = as_item(*up.on_message(request(1), t0 + 20ms));
  CHECK(wp.command == MAV_CMD_NAV_WAYPOINT);
  CHECK(wp.x == 473980000);
  CHECK(wp.y == 85460000);
  CHECK(wp.param1 == 2.0f);  // hold time

  const auto rtl = as_item(*up.on_message(request(2), t0 + 30ms));
  CHECK(rtl.command == MAV_CMD_NAV_RETURN_TO_LAUNCH);
  // No position, so no global frame: PX4 rejects RTL with x=0 in one.
  CHECK(rtl.frame == MAV_FRAME_MISSION);
  CHECK(up.sent() == 3);

  CHECK_FALSE(up.on_message(mission_ack(MAV_MISSION_ACCEPTED), t0 + 40ms));
  CHECK(up.status() == MissionStatus::Uploaded);
}

TEST_CASE("a repeated request is answered again (lost item)", "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);
  up.on_message(request(0), t0);
  const auto again = up.on_message(request(0), t0 + 1s);
  CHECK(as_item(*again).seq == 0);
  CHECK(up.sent() == 1);
}

TEST_CASE("a rejecting ack records the reason", "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);
  up.on_message(mission_ack(MAV_MISSION_NO_SPACE), t0 + 10ms);
  CHECK(up.status() == MissionStatus::Rejected);
  CHECK(up.error() == MAV_MISSION_NO_SPACE);
}

TEST_CASE("requests from elsewhere or out of range are ignored", "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);
  CHECK_FALSE(up.on_message(request(0, /*from=*/7), t0));  // other vehicle
  CHECK_FALSE(up.on_message(request(3), t0));              // no item 3
  CHECK(up.sent() == 0);
}

TEST_CASE("nothing happens before begin", "[mission]") {
  mgs::MissionUploader up;
  CHECK_FALSE(up.on_message(request(0), t0));
  CHECK_FALSE(up.retransmit(t0 + 10s));
  CHECK(up.status() == MissionStatus::None);
}

TEST_CASE("silence resends the last message, then times out", "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);  // attempt 1: the count

  CHECK_FALSE(up.retransmit(t0 + 1s));  // not silent long enough yet
  auto t = t0;
  for (int attempt = 2; attempt <= mgs::MissionUploader::kMaxAttempts;
       ++attempt) {
    t += mgs::MissionUploader::kRetryInterval;
    const auto again = up.retransmit(t);
    REQUIRE(again);
    CHECK(again->msgid == MAVLINK_MSG_ID_MISSION_COUNT);
  }
  t += mgs::MissionUploader::kRetryInterval;
  CHECK_FALSE(up.retransmit(t));  // budget spent
  CHECK(up.status() == MissionStatus::TimedOut);
}

TEST_CASE("a vehicle request resets the retry budget", "[mission]") {
  mgs::MissionUploader up;
  up.begin(three_items(), kVehicle, 1, t0);
  auto t = t0;
  for (int i = 1; i < mgs::MissionUploader::kMaxAttempts; ++i) {
    t += mgs::MissionUploader::kRetryInterval;
    REQUIRE(up.retransmit(t));  // count resent; nearly out of attempts
  }
  up.on_message(request(0), t);  // progress: the vehicle answered
  t += mgs::MissionUploader::kRetryInterval;
  const auto again = up.retransmit(t);  // still has budget for item 0
  REQUIRE(again);
  CHECK(as_item(*again).seq == 0);
}
