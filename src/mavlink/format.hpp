#pragma once

#include <optional>
#include <string>

#include <common/mavlink.h>

namespace mgs {

// Return a one-line, human-readable summary for the telemetry messages v0.1
// cares about (HEARTBEAT, SYS_STATUS, GLOBAL_POSITION_INT, ATTITUDE), or
// std::nullopt for any other message type.
std::optional<std::string> format_message(const mavlink_message_t& msg);

}  // namespace mgs
