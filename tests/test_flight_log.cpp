#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "log/flight_log.hpp"

namespace {

std::vector<std::uint8_t> read_all(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST_CASE("flight log frames a record as big-endian timestamp + raw bytes",
          "[log]") {
  const auto dir = std::filesystem::temp_directory_path() / "mgs-test-log";
  std::filesystem::remove_all(dir);

  // A known message to record.
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                             0, 0, MAV_STATE_ACTIVE);
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> raw{};
  const unsigned raw_len = mavlink_msg_to_send_buffer(raw.data(), &msg);

  // A known wall-clock instant, chosen to use all 8 bytes and an odd low digit
  // so a truncation or endianness bug shows up.
  const std::chrono::system_clock::time_point when{
      std::chrono::microseconds(1'700'000'000'000'001LL)};
  const std::uint64_t expect_us = 1'700'000'000'000'001ULL;

  std::filesystem::path path;
  {
    mgs::FlightLog log(dir);
    REQUIRE(log.healthy());
    path = log.path();
    log.record(msg, when);
    log.flush();
  }  // destructor closes the file

  const std::vector<std::uint8_t> bytes = read_all(path);
  REQUIRE(bytes.size() == 8 + raw_len);

  // First 8 bytes: timestamp, big-endian (most significant byte first).
  std::uint64_t got_us = 0;
  for (int i = 0; i < 8; ++i) got_us = (got_us << 8) | bytes[i];
  CHECK(got_us == expect_us);

  // Remaining bytes: the raw MAVLink frame, byte-for-byte.
  CHECK(std::equal(raw.begin(), raw.begin() + raw_len, bytes.begin() + 8));

  std::filesystem::remove_all(dir);
}

TEST_CASE("recorded .tlog re-parses as the original messages, in order",
          "[log]") {
  const auto dir = std::filesystem::temp_directory_path() / "mgs-test-log-rt";
  std::filesystem::remove_all(dir);

  mavlink_message_t hb, att;
  mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                             0, 0, MAV_STATE_ACTIVE);
  mavlink_msg_attitude_pack(1, 1, &att, 123, 0.1f, 0.2f, 0.3f, 0.0f, 0.0f,
                            0.0f);

  const std::chrono::system_clock::time_point t1{
      std::chrono::microseconds(1'000'000'000'000'000LL)};
  const std::chrono::system_clock::time_point t2{
      std::chrono::microseconds(1'000'000'000'500'000LL)};

  std::filesystem::path path;
  {
    mgs::FlightLog log(dir);
    REQUIRE(log.healthy());
    path = log.path();
    log.record(hb, t1);
    log.record(att, t2);
    log.flush();
  }

  const std::vector<std::uint8_t> bytes = read_all(path);

  // Walk the file as a reader like QGroundControl would: an 8-byte big-endian
  // timestamp, then exactly one MAVLink frame, repeating to end of file.
  std::vector<std::uint64_t> stamps;
  std::vector<int> ids;
  std::size_t i = 0;
  mavlink_message_t msg;
  mavlink_status_t status;
  while (i + 8 <= bytes.size()) {
    std::uint64_t ts = 0;
    for (int b = 0; b < 8; ++b) ts = (ts << 8) | bytes[i + b];
    i += 8;
    bool done = false;
    while (i < bytes.size() && !done) {
      done = mavlink_parse_char(MAVLINK_COMM_1, bytes[i++], &msg, &status) == 1;
    }
    REQUIRE(done);
    stamps.push_back(ts);
    ids.push_back(msg.msgid);
  }

  REQUIRE(stamps.size() == 2);
  CHECK(stamps[0] == 1'000'000'000'000'000ULL);
  CHECK(stamps[1] == 1'000'000'000'500'000ULL);
  CHECK(ids[0] == MAVLINK_MSG_ID_HEARTBEAT);
  CHECK(ids[1] == MAVLINK_MSG_ID_ATTITUDE);

  std::filesystem::remove_all(dir);
}

TEST_CASE("flight log degrades instead of throwing when it cannot open",
          "[log]") {
  // Put a regular file where a directory would need to be, so creating the log
  // directory under it must fail.
  const auto blocker =
      std::filesystem::temp_directory_path() / "mgs-test-log-blocker";
  std::filesystem::remove_all(blocker);
  { std::ofstream(blocker) << "x"; }  // a regular file, not a directory

  mgs::FlightLog log(blocker / "subdir");
  CHECK_FALSE(log.healthy());
  CHECK_FALSE(log.error().empty());

  std::filesystem::remove(blocker);
}
