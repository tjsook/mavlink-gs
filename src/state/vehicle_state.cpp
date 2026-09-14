#include "state/vehicle_state.hpp"

#include <cmath>

namespace mgs {
namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
}  // namespace

void update(VehicleState& state, const mavlink_message_t& msg,
            std::chrono::steady_clock::time_point now) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);
      state.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      state.system_status = hb.system_status;
      state.custom_mode = hb.custom_mode;
      state.last_heartbeat = now;  // freshness drives connection status
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t s;
      mavlink_msg_sys_status_decode(&msg, &s);
      state.battery_volts = s.voltage_battery / 1000.0;  // mV -> V
      state.battery_amps = s.current_battery / 100.0;    // cA -> A
      state.battery_remaining = s.battery_remaining;     // already percent
      break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t p;
      mavlink_msg_global_position_int_decode(&msg, &p);
      state.latitude_deg = p.lat / 1e7;              // 1e7 degrees -> degrees
      state.longitude_deg = p.lon / 1e7;
      state.altitude_m = p.alt / 1000.0;             // mm -> m (AMSL)
      state.relative_alt_m = p.relative_alt / 1000.0;  // mm -> m (above home)
      state.has_position = true;
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t a;
      mavlink_msg_attitude_decode(&msg, &a);
      state.roll_deg = a.roll * kRadToDeg;
      state.pitch_deg = a.pitch * kRadToDeg;
      state.yaw_deg = a.yaw * kRadToDeg;
      break;
    }
    default:
      break;  // a message type we don't track; leave the snapshot unchanged
  }
}

}  // namespace mgs
