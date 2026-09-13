#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <asio.hpp>
#include <catch2/catch_test_macros.hpp>
#include <common/mavlink.h>

#include "mavlink/decoder.hpp"
#include "mavlink/format.hpp"
#include "net/udp_link.hpp"

namespace {

// Serialize a packed MAVLink message to wire bytes.
std::vector<std::byte> to_bytes(const mavlink_message_t& msg) {
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  const int len = mavlink_msg_to_send_buffer(buf.data(), &msg);
  std::vector<std::byte> out(static_cast<std::size_t>(len));
  for (int i = 0; i < len; ++i) out[i] = std::byte{buf[i]};
  return out;
}

mavlink_message_t make_heartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                             MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE);
  return msg;
}

std::span<const std::byte> as_span(const std::vector<std::byte>& v) {
  return std::span<const std::byte>(v.data(), v.size());
}

}  // namespace

TEST_CASE("decoder parses a single heartbeat", "[mavlink]") {
  const auto bytes = to_bytes(make_heartbeat());

  mgs::MavlinkDecoder decoder;
  const auto messages = decoder.parse(as_span(bytes));

  REQUIRE(messages.size() == 1);
  CHECK(messages[0].msgid == MAVLINK_MSG_ID_HEARTBEAT);

  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&messages[0], &hb);
  CHECK(hb.type == MAV_TYPE_QUADROTOR);
  CHECK(hb.system_status == MAV_STATE_ACTIVE);
}

TEST_CASE("decoder recovers attitude field values", "[mavlink]") {
  mavlink_message_t msg;
  mavlink_msg_attitude_pack(1, 1, &msg, /*time_boot_ms=*/123, /*roll=*/0.10f,
                            /*pitch=*/0.20f, /*yaw=*/0.30f, 0.0f, 0.0f, 0.0f);
  const auto bytes = to_bytes(msg);

  mgs::MavlinkDecoder decoder;
  const auto messages = decoder.parse(as_span(bytes));

  REQUIRE(messages.size() == 1);
  mavlink_attitude_t a;
  mavlink_msg_attitude_decode(&messages[0], &a);
  CHECK(std::abs(a.roll - 0.10f) < 1e-4f);
  CHECK(std::abs(a.pitch - 0.20f) < 1e-4f);
  CHECK(std::abs(a.yaw - 0.30f) < 1e-4f);
}

TEST_CASE("decoder handles two messages in one buffer", "[mavlink]") {
  const auto hb = to_bytes(make_heartbeat());

  mavlink_message_t att;
  mavlink_msg_attitude_pack(1, 1, &att, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const auto attitude = to_bytes(att);

  std::vector<std::byte> both;
  both.insert(both.end(), hb.begin(), hb.end());
  both.insert(both.end(), attitude.begin(), attitude.end());

  mgs::MavlinkDecoder decoder;
  const auto messages = decoder.parse(as_span(both));

  REQUIRE(messages.size() == 2);
  CHECK(messages[0].msgid == MAVLINK_MSG_ID_HEARTBEAT);
  CHECK(messages[1].msgid == MAVLINK_MSG_ID_ATTITUDE);
}

TEST_CASE("format_message summarizes a heartbeat", "[format]") {
  const auto line = mgs::format_message(make_heartbeat());
  REQUIRE(line.has_value());
  CHECK(line->find("HEARTBEAT") != std::string::npos);
}

TEST_CASE("format_message ignores unhandled message types", "[format]") {
  mavlink_message_t msg;
  mavlink_msg_ping_pack(1, 1, &msg, 0, 0, 0, 0);
  CHECK_FALSE(mgs::format_message(msg).has_value());
}

TEST_CASE("UdpLink receives a datagram over loopback", "[net]") {
  using asio::ip::udp;

  mgs::UdpLink link(0);  // ephemeral port chosen by the OS
  const std::uint16_t port = link.local_port();

  const auto frame = to_bytes(make_heartbeat());
  asio::io_context io;
  udp::socket tx(io, udp::endpoint(udp::v4(), 0));
  tx.send_to(asio::buffer(frame.data(), frame.size()),
             udp::endpoint(asio::ip::make_address("127.0.0.1"), port));

  const auto packet = link.receive();
  REQUIRE(packet.has_value());

  mgs::MavlinkDecoder decoder;
  const auto messages = decoder.parse(*packet);
  REQUIRE(messages.size() == 1);
  CHECK(messages[0].msgid == MAVLINK_MSG_ID_HEARTBEAT);
}
