#include "mavlink/commands.hpp"

#include <cmath>

namespace mgs {
namespace {

// Our identity on the MAVLink network. 255 is the conventional ground-station
// system id; 190 (MAV_COMP_ID_MISSIONPLANNER) marks us as a GCS component.
constexpr std::uint8_t kGcsSystemId = 255;
constexpr std::uint8_t kGcsComponentId = MAV_COMP_ID_MISSIONPLANNER;

// "Use the current value" sentinel for COMMAND_LONG float params. NaN tells the
// autopilot to keep its present yaw / position rather than slew to 0.
const float kCurrent = NAN;

}  // namespace

mavlink_message_t arm_command(std::uint8_t target_system,
                              std::uint8_t target_component, bool arm) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      kGcsSystemId, kGcsComponentId, &msg, target_system, target_component,
      MAV_CMD_COMPONENT_ARM_DISARM, /*confirmation=*/0,
      /*param1 arm=*/arm ? 1.0f : 0.0f,
      /*param2 force=*/0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  return msg;
}

mavlink_message_t takeoff_command(std::uint8_t target_system,
                                  std::uint8_t target_component,
                                  float altitude_amsl_m) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      kGcsSystemId, kGcsComponentId, &msg, target_system, target_component,
      MAV_CMD_NAV_TAKEOFF, /*confirmation=*/0,
      /*param1 min pitch=*/0.0f, 0.0f, 0.0f,
      /*param4 yaw=*/kCurrent,
      /*param5 lat=*/kCurrent, /*param6 lon=*/kCurrent,
      /*param7 altitude AMSL=*/altitude_amsl_m);
  return msg;
}

mavlink_message_t land_command(std::uint8_t target_system,
                               std::uint8_t target_component) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      kGcsSystemId, kGcsComponentId, &msg, target_system, target_component,
      MAV_CMD_NAV_LAND, /*confirmation=*/0,
      /*param1 abort alt=*/0.0f, /*param2 land mode=*/0.0f, 0.0f,
      /*param4 yaw=*/kCurrent,
      /*param5 lat=*/kCurrent, /*param6 lon=*/kCurrent,
      /*param7=*/0.0f);
  return msg;
}

mavlink_message_t rtl_command(std::uint8_t target_system,
                              std::uint8_t target_component) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      kGcsSystemId, kGcsComponentId, &msg, target_system, target_component,
      MAV_CMD_NAV_RETURN_TO_LAUNCH, /*confirmation=*/0, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f);
  return msg;
}

mavlink_message_t gcs_heartbeat() {
  mavlink_message_t msg;
  // MAV_TYPE_GCS + MAV_AUTOPILOT_INVALID is how a ground station identifies
  // itself. No mode flags; MAV_STATE_ACTIVE just says "up and running".
  mavlink_msg_heartbeat_pack(kGcsSystemId, kGcsComponentId, &msg, MAV_TYPE_GCS,
                             MAV_AUTOPILOT_INVALID, /*base_mode=*/0,
                             /*custom_mode=*/0, MAV_STATE_ACTIVE);
  return msg;
}

}  // namespace mgs
