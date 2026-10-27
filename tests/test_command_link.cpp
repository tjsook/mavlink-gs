#include <array>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#include <asio.hpp>
#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "mavlink/commands.hpp"
#include "mavlink/decoder.hpp"
#include "net/command_queue.hpp"
#include "net/udp_link.hpp"
#include "net/waker.hpp"

namespace {

std::vector<std::byte> to_bytes(const mavlink_message_t& msg) {
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  const int len = mavlink_msg_to_send_buffer(buf.data(), &msg);
  std::vector<std::byte> out(static_cast<std::size_t>(len));
  for (int i = 0; i < len; ++i) out[i] = std::byte{buf[i]};
  return out;
}

}  // namespace

TEST_CASE("UdpLink::send delivers a command to the last sender", "[net]") {
  using asio::ip::udp;

  // The "vehicle": a plain UDP socket that will both ping the link and receive.
  asio::io_context io;
  udp::socket vehicle(io, udp::endpoint(udp::v4(), 0));
  const std::uint16_t vehicle_port = vehicle.local_endpoint().port();

  mgs::UdpLink link(0);
  const std::uint16_t link_port = link.local_port();

  SECTION("send is a no-op until a vehicle has been heard from") {
    CHECK_FALSE(link.has_remote());
    CHECK_FALSE(link.send(mgs::arm_command(1, 1, true)));
  }

  SECTION("after receiving, send reaches the vehicle endpoint") {
    // Vehicle -> link: a heartbeat, so the link learns the remote endpoint.
    mavlink_message_t hb;
    mavlink_msg_heartbeat_pack(1, 1, &hb, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_PX4, 0, 0, MAV_STATE_ACTIVE);
    const auto hb_bytes = to_bytes(hb);
    vehicle.send_to(asio::buffer(hb_bytes.data(), hb_bytes.size()),
                    udp::endpoint(asio::ip::make_address("127.0.0.1"),
                                  link_port));

    REQUIRE(link.receive().has_value());
    REQUIRE(link.has_remote());

    // Link -> vehicle: an arm command. The vehicle socket must receive it.
    REQUIRE(link.send(mgs::arm_command(/*target_system=*/1,
                                       /*target_component=*/1, /*arm=*/true)));

    std::array<std::uint8_t, 2048> rx{};
    udp::endpoint from;
    const std::size_t n = vehicle.receive_from(asio::buffer(rx), from);

    mgs::MavlinkDecoder decoder;
    std::vector<std::byte> bytes(n);
    for (std::size_t i = 0; i < n; ++i) bytes[i] = std::byte{rx[i]};
    const auto msgs = decoder.parse(bytes);

    REQUIRE(msgs.size() == 1);
    CHECK(msgs[0].msgid == MAVLINK_MSG_ID_COMMAND_LONG);
    mavlink_command_long_t c;
    mavlink_msg_command_long_decode(&msgs[0], &c);
    CHECK(c.command == MAV_CMD_COMPONENT_ARM_DISARM);
  }
}

TEST_CASE("CommandQueue hands messages over in order and drains empty",
          "[net]") {
  mgs::Waker wake;
  mgs::CommandQueue q(wake);
  q.push(mgs::arm_command(1, 1, true));
  q.push(mgs::takeoff_command(1, 1, 500.0f));

  const auto first = q.drain();
  REQUIRE(first.size() == 2);
  CHECK(first[0].msgid == MAVLINK_MSG_ID_COMMAND_LONG);

  const auto second = q.drain();  // nothing left
  CHECK(second.empty());
}

TEST_CASE("CommandQueue is safe under concurrent push/drain", "[net]") {
  mgs::Waker wake;
  mgs::CommandQueue q(wake);
  constexpr int kPushers = 4;
  constexpr int kPer = 5000;

  std::vector<std::thread> pushers;
  for (int p = 0; p < kPushers; ++p) {
    pushers.emplace_back([&q] {
      for (int i = 0; i < kPer; ++i) q.push(mgs::rtl_command(1, 1));
    });
  }

  int drained = 0;
  while (drained < kPushers * kPer) {
    drained += static_cast<int>(q.drain().size());
  }
  for (auto& t : pushers) t.join();
  drained += static_cast<int>(q.drain().size());

  CHECK(drained == kPushers * kPer);  // no lost or duplicated commands
}
