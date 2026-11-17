#include "log/tlog_reader.hpp"

#include <array>
#include <cstdint>

namespace mgs {
namespace {

// Bytes in a frame after its first two (start marker and payload length),
// from the marker, the length, and (v2) the flags byte that says "signed".
std::size_t frame_rest(std::uint8_t magic, std::uint8_t payload_len,
                       std::uint8_t incompat_flags) {
  if (magic == MAVLINK_STX_MAVLINK1) {
    return 4 + payload_len + MAVLINK_NUM_CHECKSUM_BYTES;  // seq sys comp id
  }
  const std::size_t signature =
      (incompat_flags & MAVLINK_IFLAG_SIGNED) ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
  // incompat, compat, seq, sys, comp, 3-byte id
  return 8 + payload_len + MAVLINK_NUM_CHECKSUM_BYTES + signature;
}

}  // namespace

std::optional<TlogRecord> TlogReader::next() {
  for (;;) {
    std::array<unsigned char, 8> stamp{};
    if (!in_.read(reinterpret_cast<char*>(stamp.data()), stamp.size())) {
      truncated_ = in_.gcount() != 0;  // a clean end reads 0 bytes
      return std::nullopt;
    }
    std::uint64_t us = 0;
    for (const unsigned char b : stamp) us = (us << 8) | b;  // big-endian

    // Start marker and payload length; for v2 also the flags byte.
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> frame{};
    if (!in_.read(reinterpret_cast<char*>(frame.data()), 3)) {
      truncated_ = true;
      return std::nullopt;
    }
    if (frame[0] != MAVLINK_STX && frame[0] != MAVLINK_STX_MAVLINK1) {
      truncated_ = true;  // not a frame: the file is damaged; stop here
      return std::nullopt;
    }
    const std::size_t total = 2 + frame_rest(frame[0], frame[1], frame[2]);
    if (!in_.read(reinterpret_cast<char*>(frame.data()) + 3,
                  static_cast<std::streamsize>(total - 3))) {
      truncated_ = true;
      return std::nullopt;
    }

    TlogRecord rec{std::chrono::system_clock::time_point{
                       std::chrono::microseconds(us)},
                   {}};
    std::uint8_t result = MAVLINK_FRAMING_INCOMPLETE;
    for (std::size_t i = 0; i < total; ++i) {
      mavlink_status_t status{};
      result = mavlink_frame_char_buffer(&rx_, &rx_status_, frame[i],
                                         &rec.msg, &status);
    }
    if (result == MAVLINK_FRAMING_OK) return rec;
    ++skipped_;  // bad checksum or unknown message: skip to the next record
  }
}

}  // namespace mgs
