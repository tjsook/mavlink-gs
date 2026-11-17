#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "log/csv_export.hpp"
#include "log/flight_log.hpp"
#include "log/tlog_reader.hpp"
#include "mavlink/commands.hpp"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::system_clock;
const Clock::time_point t0{std::chrono::seconds(1'763'000'000)};

mavlink_message_t vehicle_heartbeat(bool armed) {
  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(
      1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
      armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0, MAV_STATE_ACTIVE);
  return m;
}

mavlink_message_t position(double lat, double lon) {
  mavlink_message_t m;
  mavlink_msg_global_position_int_pack(
      1, 1, &m, 0, static_cast<std::int32_t>(lat * 1e7),
      static_cast<std::int32_t>(lon * 1e7), 500'000, 20'000, 0, 0, 0, 0);
  return m;
}

mavlink_message_t mission_current(std::uint16_t seq) {
  mavlink_message_t m;
  mavlink_msg_mission_current_pack(1, 1, &m, seq, 5, MISSION_STATE_ACTIVE, 0,
                                   0, 0, 0);
  return m;
}

// Write `records` through FlightLog — the real writer — and return the bytes.
std::string write_tlog(
    const std::vector<std::pair<Clock::time_point, mavlink_message_t>>& records) {
  const auto dir = std::filesystem::temp_directory_path() / "mgs-test-csv";
  std::filesystem::remove_all(dir);
  std::filesystem::path path;
  {
    mgs::FlightLog log(dir);
    REQUIRE(log.healthy());
    for (const auto& [when, msg] : records) log.record(msg, when);
    log.flush();
    path = log.path();
  }
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> lines(const std::string& text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) out.push_back(line);
  return out;
}

}  // namespace

TEST_CASE("TlogReader reads back what FlightLog wrote", "[csv]") {
  std::istringstream tlog(write_tlog({{t0, vehicle_heartbeat(false)},
                                      {t0 + 250ms, position(47.4, 8.5)}}));
  mgs::TlogReader reader(tlog);

  const auto a = reader.next();
  REQUIRE(a);
  CHECK(a->time == t0);
  CHECK(a->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT);

  const auto b = reader.next();
  REQUIRE(b);
  CHECK(b->time == t0 + 250ms);
  CHECK(b->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT);

  CHECK_FALSE(reader.next());
  CHECK_FALSE(reader.truncated());
  CHECK(reader.skipped() == 0);
}

TEST_CASE("TlogReader skips a damaged frame and keeps its place", "[csv]") {
  std::string bytes = write_tlog({{t0, vehicle_heartbeat(false)},
                                  {t0 + 1s, position(47.4, 8.5)}});
  bytes[8 + 12] ^= 0xFF;  // a payload byte of the first frame: bad checksum
  std::istringstream tlog(bytes);
  mgs::TlogReader reader(tlog);

  const auto rec = reader.next();
  REQUIRE(rec);
  CHECK(rec->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
  CHECK(reader.skipped() == 1);
}

TEST_CASE("TlogReader stops cleanly at a cut-off last record", "[csv]") {
  std::string bytes = write_tlog({{t0, vehicle_heartbeat(false)},
                                  {t0 + 1s, position(47.4, 8.5)}});
  bytes.resize(bytes.size() - 5);  // a crash before the last flush finished
  std::istringstream tlog(bytes);
  mgs::TlogReader reader(tlog);

  CHECK(reader.next());
  CHECK_FALSE(reader.next());
  CHECK(reader.truncated());
}

TEST_CASE("tlog_to_csv writes one row per period of vehicle state", "[csv]") {
  // Our own GCS heartbeat (disarmed) lands between the vehicle's messages; if
  // it were folded in, the armed column would flip to 0.
  std::istringstream tlog(write_tlog({
      {t0, mgs::gcs_heartbeat()},
      {t0 + 10ms, vehicle_heartbeat(true)},  // row 1: no fix yet
      {t0 + 50ms, position(47.4, 8.5)},      // too soon: no row
      {t0 + 60ms, mgs::gcs_heartbeat()},
      {t0 + 200ms, mission_current(2)},      // row 2
  }));
  std::ostringstream csv;
  const auto result = mgs::tlog_to_csv(tlog, csv);

  CHECK(result.rows == 2);
  const auto rows = lines(csv.str());
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].rfind("time_s,utc,armed,lat_deg,lon_deg,", 0) == 0);
  CHECK(rows[1] ==
        "0.010,2025-11-13T02:13:20.010Z,1,,,,,0.00,0.00,0.00,0.00,0.00,,");
  CHECK(rows[2] ==
        "0.200,2025-11-13T02:13:20.200Z,1,47.4000000,8.5000000,500.00,20.00,"
        "0.00,0.00,0.00,0.00,0.00,,3");  // mission step is 1-based
}

TEST_CASE("tlog_to_csv does not fill a gap with copies", "[csv]") {
  std::istringstream tlog(write_tlog({{t0, vehicle_heartbeat(false)},
                                      {t0 + 245s, vehicle_heartbeat(false)}}));
  std::ostringstream csv;
  CHECK(mgs::tlog_to_csv(tlog, csv).rows == 2);
}

TEST_CASE("csv_path_for puts the CSV beside the .tlog", "[csv]") {
  CHECK(mgs::csv_path_for("logs/mavlink-gs-20261116-115710.tlog") ==
        "logs/mavlink-gs-20261116-115710.csv");
}
