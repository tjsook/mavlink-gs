#pragma once

#include <chrono>
#include <cstdint>

#include <common/mavlink.h>

namespace mgs {

// The latest known telemetry for one vehicle — a snapshot, not a stream.
//
// v0.1 turned each message straight into a printed line. A dashboard instead
// shows the *current* value of each field in a fixed place, so we keep the most
// recent value of everything we care about and overwrite it as new messages
// arrive. The UI thread reads this struct; the network thread writes it (v0.5
// guards the shared copy with a mutex — see SharedState, added later).
struct VehicleState {
  // --- HEARTBEAT: liveness + high-level mode ------------------------------
  bool          armed         = false;  // base_mode & MAV_MODE_FLAG_SAFETY_ARMED
  std::uint8_t  system_status = 0;      // MAV_STATE enum (standby, active, ...)
  std::uint32_t custom_mode   = 0;      // autopilot-specific flight mode

  // Steady-clock time the last HEARTBEAT arrived. The default (zero) value
  // means "never seen". Connection status is derived from how fresh this is.
  std::chrono::steady_clock::time_point last_heartbeat{};

  // --- SYS_STATUS: power --------------------------------------------------
  double battery_volts     = 0.0;
  double battery_amps      = 0.0;
  int    battery_remaining = -1;        // percent; -1 = unknown

  // --- GLOBAL_POSITION_INT: where it is ----------------------------------
  double latitude_deg   = 0.0;
  double longitude_deg  = 0.0;
  double altitude_m     = 0.0;          // above mean sea level
  double relative_alt_m = 0.0;          // above home
  bool   has_position   = false;        // false until the first fix arrives

  // --- ATTITUDE: how it's oriented (degrees) -----------------------------
  double roll_deg  = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg   = 0.0;
};

// Fold one decoded MAVLink message into the running snapshot. Unknown message
// types leave the state untouched. `now` is injected (not read from the clock
// inside this function) so tests can assert the heartbeat timestamp with a
// fixed, deterministic value.
void update(VehicleState& state, const mavlink_message_t& msg,
            std::chrono::steady_clock::time_point now);

}  // namespace mgs
