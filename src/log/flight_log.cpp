#include "log/flight_log.hpp"

#include <array>
#include <cstdint>
#include <ctime>

namespace mgs {

namespace {

// Build "mavlink-gs-YYYYMMDD-HHMMSS.tlog" from the current local wall-clock time.
// One file per session keeps recordings from overwriting each other.
std::string session_filename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "mavlink-gs-%Y%m%d-%H%M%S.tlog", &tm);
  return buf;
}

}  // namespace

FlightLog::FlightLog(const std::filesystem::path& dir) {
  try {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);  // harmless if it exists
    path_ = dir / session_filename();

    // binary: a .tlog is a byte stream, so no newline translation.
    out_.open(path_, std::ios::binary | std::ios::trunc);
    if (!out_) {
      error_ = "cannot open " + path_.string();
      return;
    }
    // Make a failed write (e.g. disk full) throw, so record()/flush() can catch
    // it and turn logging off instead of silently dropping data.
    out_.exceptions(std::ios::badbit);
    healthy_ = true;
  } catch (const std::exception& e) {
    error_ = e.what();
    healthy_ = false;
  }
}

void FlightLog::record(const mavlink_message_t& msg,
                       std::chrono::system_clock::time_point wall_now) {
  if (!healthy_) return;
  try {
    // Timestamp: microseconds since the Unix epoch, from wall-clock time.
    const auto us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            wall_now.time_since_epoch())
            .count());

    // Write those 8 bytes BIG-ENDIAN: most significant byte first. This is a
    // MAVLink .tlog convention, not the host's native order, so we place each
    // byte by hand rather than dumping the integer's raw memory.
    std::array<char, 8> header{};
    for (int i = 0; i < 8; ++i) {
      header[i] = static_cast<char>((us >> (56 - 8 * i)) & 0xFF);
    }
    out_.write(header.data(), header.size());

    // Then the raw on-wire frame — the same serializer UdpLink::send uses.
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> frame{};
    const unsigned len = mavlink_msg_to_send_buffer(frame.data(), &msg);
    out_.write(reinterpret_cast<const char*>(frame.data()), len);
  } catch (const std::exception& e) {
    healthy_ = false;
    error_ = e.what();
  }
}

void FlightLog::flush() {
  if (!healthy_) return;
  try {
    out_.flush();
  } catch (const std::exception& e) {
    healthy_ = false;
    error_ = e.what();
  }
}

}  // namespace mgs
