#include "net/udp_link.hpp"

#include <sys/socket.h>
#include <sys/time.h>

#include <spdlog/spdlog.h>

namespace mgs {

using asio::ip::udp;

namespace {
// How long a single receive() blocks before returning "no data". Keeps the
// caller's loop responsive to shutdown without busy-waiting.
constexpr int kPollTimeoutMs = 250;
}  // namespace

UdpLink::UdpLink(std::uint16_t port)
    : socket_(io_, udp::endpoint(udp::v4(), port)) {
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = kPollTimeoutMs * 1000;
  setsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  spdlog::info("listening for MAVLink on udp://0.0.0.0:{}",
               socket_.local_endpoint().port());
}

std::optional<std::span<const std::byte>> UdpLink::receive() {
  asio::error_code ec;
  const std::size_t n =
      socket_.receive_from(asio::buffer(buffer_), remote_, 0, ec);
  if (ec) {
    // Normal, non-fatal cases that just mean "no datagram this time":
    //  - would_block / try_again: the poll timeout elapsed.
    //  - interrupted (EINTR): a signal (e.g. Ctrl+C) interrupted the syscall.
    // Returning nullopt lets the caller's loop re-check its shutdown flag.
    if (ec == asio::error::would_block || ec == asio::error::try_again ||
        ec == asio::error::interrupted) {
      return std::nullopt;
    }
    throw asio::system_error(ec);
  }
  return std::span<const std::byte>(buffer_.data(), n);
}

bool UdpLink::send(const mavlink_message_t& msg) {
  if (!has_remote()) return false;  // never heard from a vehicle; nowhere to go

  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buf{};
  const unsigned len = mavlink_msg_to_send_buffer(buf.data(), &msg);

  asio::error_code ec;
  socket_.send_to(asio::buffer(buf.data(), len), remote_, 0, ec);
  if (ec) {
    spdlog::warn("command send failed: {}", ec.message());
    return false;
  }
  return true;
}

}  // namespace mgs
