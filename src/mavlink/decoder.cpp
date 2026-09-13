#include "mavlink/decoder.hpp"

#include <cstdint>

namespace mgs {

std::vector<mavlink_message_t> MavlinkDecoder::parse(
    std::span<const std::byte> bytes) {
  std::vector<mavlink_message_t> messages;
  for (const std::byte b : bytes) {
    if (mavlink_parse_char(MAVLINK_COMM_0, std::to_integer<std::uint8_t>(b),
                           &msg_, &status_)) {
      messages.push_back(msg_);
    }
  }
  return messages;
}

}  // namespace mgs
