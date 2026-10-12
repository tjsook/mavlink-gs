#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <asio.hpp>
#include <common/mavlink.h>

namespace mgs {

// Minimal blocking UDP receiver/sender for MAVLink datagrams.
//
// v0.1 is synchronous. receive() blocks up to a short internal timeout so the
// caller's loop can react to shutdown between datagrams. We record the sender's
// endpoint on receive; v0.8 uses it as the target for send() (command sending).
//
// Thread-safety: the socket is owned by one thread (the network thread). Both
// receive() and send() run there, so the socket is never touched concurrently.
// Commands cross from the UI thread through a CommandQueue, not this object.
class UdpLink {
 public:
  // Bind a UDP socket to 0.0.0.0:<port> and start listening. Port 0 asks the
  // OS to choose a free port (see local_port()).
  explicit UdpLink(std::uint16_t port);

  // Wait up to the internal poll timeout for a datagram. Returns a view over
  // the received bytes (valid until the next call), or std::nullopt if the
  // timeout elapsed with no data.
  std::optional<std::span<const std::byte>> receive();

  // Serialize and send a MAVLink message to the most recent remote endpoint.
  // Returns false if no vehicle has been heard from yet (no known target) or
  // if the send fails; true on success.
  bool send(const mavlink_message_t& msg);

  // Whether a vehicle endpoint is known yet (set by the first receive()).
  bool has_remote() const noexcept { return remote_.port() != 0; }

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
