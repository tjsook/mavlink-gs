#pragma once

#include <cstdint>

#include <common/mavlink.h>

namespace mgs {

// Named builders for the COMMAND_LONG messages v0.8 can send.
//
// Each returns a ready-to-send mavlink_message_t addressed to the vehicle
// (target_system / target_component, taken from its HEARTBEAT). They are pure:
// no socket, no shared state — they only pack a message, so each is unit
// testable by inspecting the packed fields.
//
// The builders stamp our own identity as a ground control station (see
// kGcsSystemId / kGcsComponentId below) into the message source fields.

// Our identity on the MAVLink network. 255 is the conventional ground-station
// system id; 190 (MAV_COMP_ID_MISSIONPLANNER) marks us as a GCS component.
inline constexpr std::uint8_t kGcsSystemId = 255;
inline constexpr std::uint8_t kGcsComponentId = MAV_COMP_ID_MISSIONPLANNER;

// MAV_CMD_COMPONENT_ARM_DISARM: arm (true) or disarm (false) the vehicle.
mavlink_message_t arm_command(std::uint8_t target_system,
                              std::uint8_t target_component, bool arm);

// MAV_CMD_NAV_TAKEOFF: climb to `altitude_amsl_m` (metres above mean sea level)
// at the current horizontal position. The altitude is AMSL, not relative — the
// caller adds the desired height to the vehicle's current altitude.
mavlink_message_t takeoff_command(std::uint8_t target_system,
                                  std::uint8_t target_component,
                                  float altitude_amsl_m);

// MAV_CMD_NAV_LAND: land at the current horizontal position.
mavlink_message_t land_command(std::uint8_t target_system,
                               std::uint8_t target_component);

// MAV_CMD_NAV_RETURN_TO_LAUNCH: fly back to the launch point and land.
mavlink_message_t rtl_command(std::uint8_t target_system,
                              std::uint8_t target_component);

// MAV_CMD_MISSION_START: fly the uploaded mission from its first item to its
// last. The vehicle must already hold a mission (see MissionUploader).
mavlink_message_t mission_start_command(std::uint8_t target_system,
                                        std::uint8_t target_component);

// HEARTBEAT announcing us as a ground control station. Not a command, but it
// shares the GCS source identity. A GCS must send this at about 1 Hz; without
// it the autopilot reports "no connection to the GCS" and refuses to arm.
mavlink_message_t gcs_heartbeat();

}  // namespace mgs
