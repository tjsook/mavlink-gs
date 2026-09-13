#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <asio.hpp>

namespace mgs {

// Minimal blocking UDP receiver for MAVLink datagrams.
//
// v0.1 is synchronous. receive() blocks up to a short internal timeout so the
// caller's loop can react to shutdown between datagrams. We also record the
// sender's endpoint so a later version can reply to it (command sending, v0.8).
class UdpLink {
 public:
  // Bind a UDP socket to 0.0.0.0:<port> and start listening. Port 0 asks the
  // OS to choose a free port (see local_port()).
  explicit UdpLink(std::uint16_t port);

  // Wait up to the internal poll timeout for a datagram. Returns a view over
  // the received bytes (valid until the next call), or std::nullopt if the
  // timeout elapsed with no data.
  std::optional<std::span<const std::byte>> receive();

  // The most recent sender (valid after the first successful receive()).
  const asio::ip::udp::endpoint& remote() const noexcept { return remote_; }

  // The local port actually bound (useful when constructed with port 0).
  std::uint16_t local_port() const { return socket_.local_endpoint().port(); }

 private:
  asio::io_context io_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint remote_;
  std::array<std::byte, 2048> buffer_{};
};

}  // namespace mgs
