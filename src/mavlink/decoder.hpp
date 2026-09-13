#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include <common/mavlink.h>

namespace mgs {

// Incrementally parses a MAVLink v2 byte stream into complete messages.
//
// Bytes arrive in datagram-sized chunks that may contain zero, one, or several
// messages (and messages may straddle chunk boundaries). parse() feeds each
// byte through the MAVLink state machine and returns whatever completed.
class MavlinkDecoder {
 public:
  // Feed a chunk of bytes; return every complete message found within it.
  std::vector<mavlink_message_t> parse(std::span<const std::byte> bytes);

 private:
  mavlink_message_t msg_{};
  mavlink_status_t status_{};
};

}  // namespace mgs
