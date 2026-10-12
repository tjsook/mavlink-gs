#include <chrono>

#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "mavlink/command_tracker.hpp"
#include "state/vehicle_state.hpp"

namespace {

using namespace std::chrono_literals;
const auto t0 = std::chrono::steady_clock::time_point{} + 100s;

mavlink_command_ack_t ack(std::uint16_t command, std::uint8_t result) {
  mavlink_command_ack_t a{};
  a.command = command;
  a.result = result;
  return a;
}

constexpr std::uint16_t kArm = MAV_CMD_COMPONENT_ARM_DISARM;

}  // namespace

TEST_CASE("a freshly begun command is pending", "[tracker]") {
  mgs::CommandTracker t;
  CHECK(t.status() == mgs::CommandStatus::None);

  t.begin(kArm, t0);
  CHECK(t.status() == mgs::CommandStatus::Pending);
  CHECK(t.command() == kArm);
}

TEST_CASE("a matching accepted ack accepts the command", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(kArm, t0);
  t.on_ack(ack(kArm, MAV_RESULT_ACCEPTED));
  CHECK(t.status() == mgs::CommandStatus::Accepted);
}

TEST_CASE("an ack for a different command is ignored", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(kArm, t0);
  t.on_ack(ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED));
  CHECK(t.status() == mgs::CommandStatus::Pending);  // still waiting on ours
}

TEST_CASE("a rejecting ack marks the command rejected", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(kArm, t0);
  t.on_ack(ack(kArm, MAV_RESULT_DENIED));
  CHECK(t.status() == mgs::CommandStatus::Rejected);
}

TEST_CASE("an unacked command resends, then times out", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(kArm, t0);  // attempt 1 (caller sent it)

  CHECK_FALSE(t.should_resend(t0 + 500ms));  // too soon
  CHECK(t.should_resend(t0 + 1s));           // attempt 2
  CHECK_FALSE(t.should_resend(t0 + 1s));     // timer just reset
  CHECK(t.should_resend(t0 + 2s));           // attempt 3
  CHECK_FALSE(t.should_resend(t0 + 3s));     // budget spent
  CHECK(t.status() == mgs::CommandStatus::TimedOut);
}

TEST_CASE("an acknowledged command never resends", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(kArm, t0);
  t.on_ack(ack(kArm, MAV_RESULT_ACCEPTED));
  CHECK_FALSE(t.should_resend(t0 + 2s));  // already heard back
}

TEST_CASE("in-progress stays pending but stops the retransmits", "[tracker]") {
  mgs::CommandTracker t;
  t.begin(MAV_CMD_NAV_TAKEOFF, t0);
  t.on_ack(ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT_IN_PROGRESS));
  CHECK(t.status() == mgs::CommandStatus::Pending);  // vehicle working on it
  CHECK_FALSE(t.should_resend(t0 + 2s));             // but it heard us
}
