#include <poll.h>

#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "mavlink/commands.hpp"
#include "net/command_queue.hpp"
#include "net/udp_link.hpp"
#include "net/waker.hpp"

namespace {

using namespace std::chrono_literals;

// Would poll() return immediately for this fd? (Timeout 0: just ask, no sleep.)
bool readable(int fd) {
  pollfd p{fd, POLLIN, 0};
  return ::poll(&p, 1, 0) == 1;
}

}  // namespace

TEST_CASE("Waker: notify makes the fd ready, clear makes it idle", "[net]") {
  mgs::Waker wake;
  CHECK_FALSE(readable(wake.fd()));

  wake.notify();
  CHECK(readable(wake.fd()));

  wake.clear();
  CHECK_FALSE(readable(wake.fd()));
}

TEST_CASE("Waker: one clear empties many notifies", "[net]") {
  mgs::Waker wake;
  for (int i = 0; i < 10; ++i) wake.notify();
  wake.clear();
  CHECK_FALSE(readable(wake.fd()));  // else poll() would spin on stale bytes
}

TEST_CASE("Waker: notify never blocks, even when the pipe is full", "[net]") {
  mgs::Waker wake;
  // A pipe holds ~16-64 KiB. Far more notifies than that must still return.
  for (int i = 0; i < 200'000; ++i) wake.notify();
  CHECK(readable(wake.fd()));
  wake.clear();
  CHECK_FALSE(readable(wake.fd()));
}

TEST_CASE("UdpLink::receive returns early when the Waker rings", "[net]") {
  mgs::UdpLink link(0);  // no one will ever send to this port
  mgs::Waker wake;
  mgs::CommandQueue commands(wake);

  // Another thread queues a command shortly after we start sleeping.
  std::thread ui([&] {
    std::this_thread::sleep_for(20ms);
    commands.push(mgs::rtl_command(1, 1));
  });

  const auto start = std::chrono::steady_clock::now();
  const auto packet = link.receive(wake.fd());
  const auto waited = std::chrono::steady_clock::now() - start;
  ui.join();

  CHECK_FALSE(packet.has_value());  // woken, not handed a datagram
  CHECK(waited < 150ms);            // well under the 250 ms poll timeout
  CHECK(commands.drain().size() == 1);
}
