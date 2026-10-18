#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <common/mavlink.h>

namespace mgs {

// Records every MAVLink message to a QGroundControl-compatible .tlog file.
//
// Format: each record is an 8-byte BIG-ENDIAN timestamp — microseconds since the
// Unix epoch, wall-clock — followed by the raw on-wire MAVLink bytes. QGround-
// Control and Mission Planner replay this format directly.
//
// Durability vs. cost: writes go to a buffer and reach the disk only on flush(),
// which the caller runs about once a second — never per message. A crash loses
// at most ~1 s of the tail. That is the deliberate trade: the hot loop never
// waits on the disk.
//
// Robustness: the log is a passenger, never the pilot. record() and flush()
// never throw; a write error turns logging off (healthy() becomes false) and the
// caller shows that on screen. A failed log must never stop telemetry or
// commands — the same rule as the network loop's receive/send duties.
class FlightLog {
 public:
  // Opens <dir>/mavlink-gs-YYYYMMDD-HHMMSS.tlog, creating <dir> if needed.
  // On any failure healthy() is false and error() explains. The constructor does
  // not throw, so a bad log directory degrades to "no logging" instead of
  // blocking startup.
  explicit FlightLog(const std::filesystem::path& dir);

  // Append one message as a .tlog record. No-op when logging is off. Never
  // throws: a write failure turns logging off.
  void record(const mavlink_message_t& msg,
              std::chrono::system_clock::time_point wall_now);

  // Push buffered bytes to the OS. Call about once a second, not per message.
  // Never throws.
  void flush();

  bool healthy() const { return healthy_; }
  const std::string& error() const { return error_; }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::ofstream out_;
  std::filesystem::path path_;
  bool healthy_ = false;
  std::string error_;
};

}  // namespace mgs
