#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <spdlog/spdlog.h>

#include "core/version.hpp"
#include "mavlink/decoder.hpp"
#include "net/udp_link.hpp"
#include "state/shared_state.hpp"
#include "state/vehicle_state.hpp"
#include "ui/dashboard.hpp"

namespace {

using namespace std::chrono_literals;

// A vehicle counts as "connected" if a HEARTBEAT arrived within this window.
// PX4 sends HEARTBEAT at 1 Hz, so 3 s tolerates ~2 missed beats before the
// indicator flips to DISCONNECTED — long enough to ignore a single drop, short
// enough to notice a real one (Decision 4).
constexpr auto kHeartbeatTimeout = 3s;

// UI redraw period. 100 ms -> 10 Hz, meeting the v0.5 "live >= 10 Hz" target.
constexpr auto kRefreshPeriod = 100ms;

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

  // Open the socket on the main thread, before the TUI takes over the screen,
  // so a bind failure (e.g. port in use) is reported plainly and we exit —
  // rather than showing a fullscreen dashboard stuck on DISCONNECTED.
  std::unique_ptr<mgs::UdpLink> link;
  try {
    link = std::make_unique<mgs::UdpLink>(port);
  } catch (const std::exception& e) {
    spdlog::error("cannot open {}: {}", connection, e.what());
    return 1;
  }

  mgs::SharedState shared;
  std::atomic<bool> stop{false};
  std::string net_error;  // read only after network.join() (see below)

  // --- Network thread: receive -> decode -> fold into shared state --------
  // Owns the socket (moved in). receive() blocks up to its internal poll
  // timeout, so the loop re-checks `stop` a few times a second on shutdown.
  std::thread network([&, link = std::move(link)]() mutable {
    mgs::MavlinkDecoder decoder;
    try {
      while (!stop) {
        const auto packet = link->receive();
        if (!packet) continue;  // poll timeout; loop back and re-check stop
        const auto now = std::chrono::steady_clock::now();
        for (const auto& msg : decoder.parse(*packet)) {
          shared.modify(
              [&](mgs::VehicleState& s) { mgs::update(s, msg, now); });
        }
      }
    } catch (const std::exception& e) {
      net_error = e.what();  // surfaced after the UI loop ends
    }
  });

  // --- UI (this thread): render the latest snapshot -----------------------
  auto screen = ftxui::ScreenInteractive::Fullscreen();

  auto view = ftxui::Renderer([&] {
    const mgs::VehicleState snap = shared.snapshot();
    const bool connected =
        snap.last_heartbeat != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - snap.last_heartbeat <
            kHeartbeatTimeout;
    return mgs::render_dashboard(snap, connected);
  });

  auto root = ftxui::CatchEvent(view, [&](const ftxui::Event& e) {
    if (e == ftxui::Event::Character('q') || e == ftxui::Event::Escape) {
      screen.Exit();
      return true;
    }
    return false;
  });

  // --- Refresh ticker: wake the UI 10x/second -----------------------------
  // FTXUI only repaints in response to an event. Telemetry arrives on the
  // network thread, which is not an event, so without this the screen would
  // freeze between keystrokes. A custom event on a timer forces the redraw.
  std::thread ticker([&] {
    while (!stop) {
      std::this_thread::sleep_for(kRefreshPeriod);
      screen.PostEvent(ftxui::Event::Custom);
    }
  });

  screen.Loop(root);  // blocks until 'q', Esc, or Ctrl+C

  // --- Shutdown -----------------------------------------------------------
  stop = true;
  ticker.join();
  network.join();
  if (!net_error.empty()) {
    spdlog::error("network link stopped: {}", net_error);
  }
  spdlog::info("mavlink-gs shut down cleanly");
  return 0;
}
