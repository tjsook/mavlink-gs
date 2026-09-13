#include "mavlink/format.hpp"

#include <cmath>

#include <spdlog/fmt/fmt.h>

namespace mgs {
namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
}  // namespace

std::optional<std::string> format_message(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);
      return fmt::format(
          "HEARTBEAT   type={} autopilot={} base_mode={:#04x} state={}",
          static_cast<int>(hb.type), static_cast<int>(hb.autopilot),
          static_cast<unsigned>(hb.base_mode),
          static_cast<int>(hb.system_status));
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t s;
      mavlink_msg_sys_status_decode(&msg, &s);
      return fmt::format(
          "SYS_STATUS  batt={:.2f}V current={:.2f}A remaining={}%",
          s.voltage_battery / 1000.0, s.current_battery / 100.0,
          static_cast<int>(s.battery_remaining));
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t p;
      mavlink_msg_global_position_int_decode(&msg, &p);
      return fmt::format(
          "POSITION    lat={:.7f} lon={:.7f} alt={:.1f}m rel_alt={:.1f}m",
          p.lat / 1e7, p.lon / 1e7, p.alt / 1000.0, p.relative_alt / 1000.0);
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t a;
      mavlink_msg_attitude_decode(&msg, &a);
      return fmt::format(
          "ATTITUDE    roll={:.1f} pitch={:.1f} yaw={:.1f} (deg)",
          a.roll * kRadToDeg, a.pitch * kRadToDeg, a.yaw * kRadToDeg);
    }
    default:
      return std::nullopt;
  }
}

}  // namespace mgs
