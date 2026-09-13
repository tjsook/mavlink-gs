#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include "core/version.hpp"
#include "mavlink/decoder.hpp"
#include "mavlink/format.hpp"
#include "net/udp_link.hpp"

namespace {

// Set by SIGINT (Ctrl+C) so the main loop can exit cleanly.
std::atomic<bool> g_stop{false};
void handle_sigint(int /*signum*/) { g_stop = true; }

// Per-message-type print interval — PX4 sends some messages very fast, so we
// cap each type to a readable rate for CLI output.
constexpr auto kThrottle = std::chrono::milliseconds(500);

// Extract the port from a "udp://host:port" string; fall back on any failure.
std::uint16_t parse_port(const std::string& url, std::uint16_t fallback) {
  const auto colon = url.rfind(':');
  if (colon == std::string::npos) return fallback;
  try {
    return static_cast<std::uint16_t>(std::stoi(url.substr(colon + 1)));
  } catch (...) {
    return fallback;
  }
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"mavlink-gs — terminal-based MAVLink ground station"};
  app.set_version_flag("--version", mgs::version());

  std::string connection = "udp://127.0.0.1:14550";
  app.add_option("--connect", connection,
                 "Connection URL, e.g. udp://127.0.0.1:14550");
  CLI11_PARSE(app, argc, argv);

  const std::uint16_t port = parse_port(connection, 14550);
  std::signal(SIGINT, handle_sigint);

  spdlog::info("mavlink-gs {} starting", mgs::version());

  mgs::UdpLink link(port);
  mgs::MavlinkDecoder decoder;
  std::unordered_map<std::uint32_t, std::chrono::steady_clock::time_point>
      last_print;

  while (!g_stop) {
    const auto packet = link.receive();
    if (!packet) continue;  // poll timeout; loop back and re-check g_stop

    for (const auto& msg : decoder.parse(*packet)) {
      const auto line = mgs::format_message(msg);
      if (!line) continue;

      const auto now = std::chrono::steady_clock::now();
      auto& last = last_print[msg.msgid];
      if (last.time_since_epoch().count() != 0 && now - last < kThrottle) {
        continue;
      }
      last = now;
      std::cout << *line << '\n';
    }
  }

  spdlog::info("disconnected, shutting down");
  return 0;
}
