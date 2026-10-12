#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <common/mavlink.h>

#include "mavlink/commands.hpp"

using Catch::Matchers::WithinAbs;

namespace {

// Decode a built message back into a COMMAND_LONG for inspection.
mavlink_command_long_t decode(const mavlink_message_t& msg) {
  mavlink_command_long_t c;
  mavlink_msg_command_long_decode(&msg, &c);
  return c;
}

}  // namespace

TEST_CASE("arm_command targets the vehicle and sets param1", "[commands]") {
  const auto msg = mgs::arm_command(/*target_system=*/1, /*target_component=*/1,
                                    /*arm=*/true);
  const auto c = decode(msg);

  CHECK(c.target_system == 1);
  CHECK(c.target_component == 1);
  CHECK(c.command == MAV_CMD_COMPONENT_ARM_DISARM);
  CHECK_THAT(c.param1, WithinAbs(1.0, 1e-6));  // 1 = arm

  const auto disarm = decode(mgs::arm_command(1, 1, /*arm=*/false));
  CHECK_THAT(disarm.param1, WithinAbs(0.0, 1e-6));  // 0 = disarm
}

TEST_CASE("takeoff_command carries AMSL altitude and 'use current' position",
          "[commands]") {
  const auto c = decode(mgs::takeoff_command(1, 1, /*altitude_amsl_m=*/513.5f));

  CHECK(c.command == MAV_CMD_NAV_TAKEOFF);
  CHECK_THAT(c.param7, WithinAbs(513.5, 1e-3));  // altitude AMSL
  // lat/lon must be NaN ("use current"), never 0 (which means lat/lon 0,0).
  CHECK(std::isnan(c.param5));
  CHECK(std::isnan(c.param6));
}

TEST_CASE("land_command lands at current position", "[commands]") {
  const auto c = decode(mgs::land_command(7, 1));

  CHECK(c.target_system == 7);
  CHECK(c.command == MAV_CMD_NAV_LAND);
  CHECK(std::isnan(c.param5));
  CHECK(std::isnan(c.param6));
}

TEST_CASE("rtl_command issues return-to-launch", "[commands]") {
  const auto c = decode(mgs::rtl_command(1, 1));
  CHECK(c.command == MAV_CMD_NAV_RETURN_TO_LAUNCH);
}

TEST_CASE("built commands carry our GCS identity as the source", "[commands]") {
  const auto msg = mgs::arm_command(1, 1, true);
  CHECK(msg.sysid == 255);                        // conventional GCS system id
  CHECK(msg.compid == MAV_COMP_ID_MISSIONPLANNER);
}
