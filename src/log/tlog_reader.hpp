#pragma once

#include <chrono>
#include <cstddef>
#include <istream>
#include <optional>

#include <common/mavlink.h>

namespace mgs {

// One record of a .tlog: when it was logged, and the message.
struct TlogRecord {
  std::chrono::system_clock::time_point time;
  mavlink_message_t msg;
};

// Reads a .tlog (see FlightLog for the format) back one record at a time.
//
// Each record is an 8-byte big-endian microsecond timestamp, then one MAVLink
// frame. The frame's own header says how long it is, so the reader takes
// exactly that many bytes and stays in step with the records even past a frame
// that fails its checksum (that record is skipped and counted).
//
// It keeps its own parser state rather than using MavlinkDecoder: that one
// parses on the library's shared MAVLINK_COMM_0 buffers, which belong to the
// live network thread.
class TlogReader {
 public:
  explicit TlogReader(std::istream& in) : in_(in) {}

  // The next good record, or nullopt at the end of the file. A file cut off
  // mid-record (e.g. a crash before the last flush) ends early: truncated().
  std::optional<TlogRecord> next();

  std::size_t skipped() const { return skipped_; }  // frames with a bad CRC
  bool truncated() const { return truncated_; }

 private:
  std::istream& in_;
  mavlink_message_t rx_{};       // parser working buffer
  mavlink_status_t rx_status_{};
  std::size_t skipped_ = 0;
  bool truncated_ = false;
};

}  // namespace mgs
