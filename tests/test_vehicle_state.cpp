#include <chrono>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/mavlink.h>

#include "state/vehicle_state.hpp"

using Catch::Matchers::WithinAbs;

namespace {

// A fixed, arbitrary instant so heartbeat-timestamp assertions are exact.
const auto kNow = std::chrono::steady_clock::time_point{} +
                  std::chrono::seconds(1000);

mavlink_message_t pack_heartbeat(std::uint8_t base_mode,
                                 std::uint8_t system_status) {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                             base_mode, /*custom_mode=*/0, system_status);
  return msg;
}

}  // namespace

TEST_CASE("heartbeat sets armed flag and freshness stamp", "[state]") {
  mgs::VehicleState state;
  const auto msg = pack_heartbeat(MAV_MODE_FLAG_SAFETY_ARMED, MAV_STATE_ACTIVE);

  mgs::update(state, msg, kNow);

  CHECK(state.armed);
  CHECK(state.system_status == MAV_STATE_ACTIVE);
  CHECK(state.last_heartbeat == kNow);
}

TEST_CASE("sys_status is scaled into volts, amps, percent", "[state]") {
  mgs::VehicleState state;
  mavlink_message_t msg;
  mavlink_msg_sys_status_pack(
      1, 1, &msg, /*sensors_present=*/0, /*sensors_enabled=*/0,
      /*sensors_health=*/0, /*load=*/0, /*voltage_battery=*/11700,
      /*current_battery=*/850, /*battery_remaining=*/76, /*drop_rate_comm=*/0,
      /*errors_comm=*/0, 0, 0, 0, 0, /*present_ext=*/0, /*enabled_ext=*/0,
      /*health_ext=*/0);

  mgs::update(state, msg, kNow);

  CHECK_THAT(state.battery_volts, WithinAbs(11.7, 1e-6));
  CHECK_THAT(state.battery_amps, WithinAbs(8.5, 1e-6));
  CHECK(state.battery_remaining == 76);
}

TEST_CASE("global position is scaled into degrees and metres", "[state]") {
  mgs::VehicleState state;
  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(
      1, 1, &msg, /*time_boot_ms=*/0, /*lat=*/377749000, /*lon=*/-1224194000,
      /*alt=*/100000, /*relative_alt=*/25000, 0, 0, 0, /*hdg=*/0);

  mgs::update(state, msg, kNow);

  CHECK(state.has_position);
  CHECK_THAT(state.latitude_deg, WithinAbs(37.7749, 1e-4));
  CHECK_THAT(state.longitude_deg, WithinAbs(-122.4194, 1e-4));
  CHECK_THAT(state.altitude_m, WithinAbs(100.0, 1e-6));
  CHECK_THAT(state.relative_alt_m, WithinAbs(25.0, 1e-6));
}

TEST_CASE("attitude is converted from radians to degrees", "[state]") {
  mgs::VehicleState state;
  mavlink_message_t msg;
  mavlink_msg_attitude_pack(1, 1, &msg, /*time_boot_ms=*/0,
                            /*roll=*/1.5707963f,  // pi/2 -> 90 deg
                            /*pitch=*/0.0f, /*yaw=*/-0.7853982f,  // -45 deg
                            0.0f, 0.0f, 0.0f);

  mgs::update(state, msg, kNow);

  CHECK_THAT(state.roll_deg, WithinAbs(90.0, 1e-3));
  CHECK_THAT(state.pitch_deg, WithinAbs(0.0, 1e-3));
  CHECK_THAT(state.yaw_deg, WithinAbs(-45.0, 1e-3));
}

TEST_CASE("an untracked message leaves the snapshot unchanged", "[state]") {
  mgs::VehicleState state;
  mavlink_message_t msg;
  mavlink_msg_ping_pack(1, 1, &msg, 0, 0, 0, 0);

  mgs::update(state, msg, kNow);

  CHECK_FALSE(state.armed);
  CHECK_FALSE(state.has_position);
  CHECK(state.battery_remaining == -1);
}
