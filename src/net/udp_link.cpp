#include "net/udp_link.hpp"

#include <poll.h>

#include <cerrno>
#include <system_error>

#include <spdlog/spdlog.h>

namespace mgs {

using asio::ip::udp;

namespace {
// How long a single receive() sleeps when nothing happens at all. Commands and
// shutdown no longer wait on this (the Waker cuts the sleep short); it only
// bounds how late the caller's timers (heartbeat, retry, flush) can fire on a
// silent link.
constexpr int kPollTimeoutMs = 250;
}  // namespace

UdpLink::UdpLink(std::uint16_t port)
    : socket_(io_, udp::endpoint(udp::v4(), port)) {
  // poll() tells us when to read, so the read itself must never block.
  socket_.non_blocking(true);
  spdlog::info("listening for MAVLink on udp://0.0.0.0:{}",
               socket_.local_endpoint().port());
}

std::optional<std::span<const std::byte>> UdpLink::receive(int wake_fd) {
  // Sleep on both file descriptors at once. poll() ignores an entry whose fd
  // is negative, so wake_fd == -1 watches the socket alone.
  std::array<pollfd, 2> fds{{
      {socket_.native_handle(), POLLIN, 0},
      {wake_fd, POLLIN, 0},
  }};
  if (::poll(fds.data(), fds.size(), kPollTimeoutMs) < 0) {
    if (errno == EINTR) return std::nullopt;  // a signal; let the loop re-check
    throw std::system_error(errno, std::generic_category(), "poll");
  }
  // Socket quiet: we timed out or were woken. Either way, no datagram. Any
  // socket event (POLLIN or POLLERR) falls through, so receive_from() reports
  // the error instead of poll() spinning on it.
  if (fds[0].revents == 0) return std::nullopt;

  asio::error_code ec;
  const std::size_t n =
      socket_.receive_from(asio::buffer(buffer_), remote_, 0, ec);
  if (ec) {
    // Normal, non-fatal cases that just mean "no datagram this time":
    //  - would_block / try_again: poll() said ready, but nothing was there.
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
