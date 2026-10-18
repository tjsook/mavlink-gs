#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <spdlog/spdlog.h>

#include "core/version.hpp"
#include "log/flight_log.hpp"
#include "mavlink/command_tracker.hpp"
#include "mavlink/commands.hpp"
#include "mavlink/decoder.hpp"
#include "net/command_queue.hpp"
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

// How often the flight log is flushed to disk. Buffered in between, so the hot
// loop never waits on the disk; a crash loses at most this much of the tail.
constexpr auto kFlushInterval = 1s;

// Height above the vehicle's current altitude to request on takeoff.
constexpr float kTakeoffHeightM = 2.5f;

// The commands the palette can issue.
enum class Cmd { Arm, Disarm, Takeoff, Land, Rtl };

// Dangerous commands get a y/n confirmation; the rest fire immediately.
bool needs_confirm(Cmd c) {
  return c == Cmd::Arm || c == Cmd::Takeoff || c == Cmd::Rtl;
}

const char* cmd_name(Cmd c) {
  switch (c) {
    case Cmd::Arm: return "ARM";
    case Cmd::Disarm: return "DISARM";
    case Cmd::Takeoff: return "TAKEOFF";
    case Cmd::Land: return "LAND";
    case Cmd::Rtl: return "RTL";
  }
  return "";
}

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

  std::string log_dir = "logs";
  app.add_option("--log-dir", log_dir,
                 "Directory for .tlog flight recordings")
      ->capture_default_str();
  bool no_log = false;
  app.add_flag("--no-log", no_log, "Disable flight-log recording");

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

  // Open the flight log before the TUI, so the "recording to <path>" line (or a
  // failure notice) is readable on the terminal. A failed open does not stop us
  // — logging is a passenger, never the pilot.
  std::unique_ptr<mgs::FlightLog> flight_log;
  if (!no_log) {
    flight_log = std::make_unique<mgs::FlightLog>(log_dir);
    if (flight_log->healthy()) {
      spdlog::info("recording flight log to {}", flight_log->path().string());
    } else {
      spdlog::warn("flight logging disabled: {}", flight_log->error());
    }
  }

  mgs::SharedState shared;
  mgs::CommandQueue commands;  // UI thread pushes; network thread sends
  std::atomic<bool> stop{false};
  std::string net_error;  // read only after network.join() (see below)

  // --- Network thread: receive -> decode -> fold into shared state --------
  // Owns the socket (moved in). receive() blocks up to its internal poll
  // timeout, so the loop re-checks `stop` a few times a second on shutdown.
  std::thread network([&, link = std::move(link),
                       log = std::move(flight_log)]() mutable {
    mgs::MavlinkDecoder decoder;
    mgs::CommandTracker tracker;     // one in-flight command at a time
    mavlink_message_t in_flight{};   // the message to resend on retry
    mgs::CommandStatus published = mgs::CommandStatus::None;
    mgs::LogStatus log_published = mgs::LogStatus::Off;
    auto last_flush = std::chrono::steady_clock::now();

    // The log timestamp is wall-clock (system_clock), not the steady_clock the
    // loop uses for timeouts: a .tlog means nothing to QGroundControl unless its
    // times are real "since 1970" times. Two clocks, two jobs.
    const auto log_now = [] { return std::chrono::system_clock::now(); };

    try {
      while (!stop) {
        const auto now = std::chrono::steady_clock::now();

        // receive() blocks up to its poll timeout, then returns nullopt. We do
        // not `continue` on a timeout: command handling below must run every
        // pass, not only when a datagram happened to arrive.
        if (const auto packet = link->receive()) {
          for (const auto& msg : decoder.parse(*packet)) {
            shared.modify(
                [&](mgs::VehicleState& s) { mgs::update(s, msg, now); });
            if (log) log->record(msg, log_now());  // record what we received
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
              mavlink_command_ack_t ack;
              mavlink_msg_command_ack_decode(&msg, &ack);
              tracker.on_ack(ack);  // resolve/accept/reject the in-flight cmd
            }
          }
        }

        // Send commands the UI queued since the last pass. Drained here, on the
        // socket-owning thread, so the UI thread never touches the socket.
        for (const auto& cmd : commands.drain()) {
          if (link->send(cmd)) {
            mavlink_command_long_t cl;
            mavlink_msg_command_long_decode(&cmd, &cl);
            tracker.begin(cl.command, now);  // count this send as attempt 1
            in_flight = cmd;
            if (log) log->record(cmd, log_now());  // record what we sent, too
          }
        }

        // Retransmit the in-flight command if its ack is overdue.
        if (tracker.should_resend(now)) {
          if (link->send(in_flight) && log) log->record(in_flight, log_now());
        }

        // Flush the log on a timer, not per message: the disk stays off the hot
        // path. A write failure here turns logging off inside the recorder.
        if (log && now - last_flush >= kFlushInterval) {
          log->flush();
          last_flush = now;
        }

        // Publish command status to the UI only when it changes.
        if (tracker.status() != published) {
          published = tracker.status();
          const auto id = tracker.command();
          shared.modify([&](mgs::VehicleState& s) {
            s.last_command_status = published;
            s.last_command_id = id;
          });
        }

        // Publish log status only when it changes (Recording -> Failed on a bad
        // write), so the dashboard can warn without us touching it per pass.
        const mgs::LogStatus log_status =
            !log ? mgs::LogStatus::Off
                 : (log->healthy() ? mgs::LogStatus::Recording
                                   : mgs::LogStatus::Failed);
        if (log_status != log_published) {
          log_published = log_status;
          shared.modify(
              [&](mgs::VehicleState& s) { s.log_status = log_status; });
        }
      }
    } catch (const std::exception& e) {
      net_error = e.what();  // surfaced after the UI loop ends
    }
  });

  // --- UI (this thread): render the latest snapshot -----------------------
  auto screen = ftxui::ScreenInteractive::Fullscreen();

  // UI-local command state. Only the UI thread touches these, so no lock.
  std::optional<Cmd> pending_confirm;  // a dangerous command awaiting y/n
  std::string note;                    // transient feedback (e.g. "no vehicle")

  // Build the chosen command from the latest snapshot and queue it for sending.
  // Refuses (with a note) if we can't address or safely issue it yet.
  auto issue = [&](Cmd c) {
    const mgs::VehicleState snap = shared.snapshot();
    if (!snap.identified()) {
      note = "no vehicle yet — command ignored";
      return;
    }
    const std::uint8_t ts = snap.system_id;
    const std::uint8_t tc = snap.component_id;
    switch (c) {
      case Cmd::Arm:
        commands.push(mgs::arm_command(ts, tc, true));
        break;
      case Cmd::Disarm:
        commands.push(mgs::arm_command(ts, tc, false));
        break;
      case Cmd::Takeoff:
        if (!snap.has_position) {
          note = "no position fix — cannot take off";
          return;
        }
        commands.push(mgs::takeoff_command(
            ts, tc, static_cast<float>(snap.altitude_m) + kTakeoffHeightM));
        break;
      case Cmd::Land:
        commands.push(mgs::land_command(ts, tc));
        break;
      case Cmd::Rtl:
        commands.push(mgs::rtl_command(ts, tc));
        break;
    }
    note.clear();
  };

  auto view = ftxui::Renderer([&] {
    const mgs::VehicleState snap = shared.snapshot();
    const bool connected =
        snap.last_heartbeat != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - snap.last_heartbeat <
            kHeartbeatTimeout;
    const std::string prompt =
        pending_confirm
            ? std::string(cmd_name(*pending_confirm)) + "? confirm (y/n)"
            : "";
    return mgs::render_dashboard(snap, connected, prompt, note);
  });

  auto root = ftxui::CatchEvent(view, [&](const ftxui::Event& e) {
    // While confirming a dangerous command, y/n resolves it and nothing else.
    if (pending_confirm) {
      if (e == ftxui::Event::Character('y')) {
        issue(*pending_confirm);
        pending_confirm.reset();
      } else if (e == ftxui::Event::Character('n') ||
                 e == ftxui::Event::Escape) {
        pending_confirm.reset();
        note = "cancelled";
      }
      return true;  // swallow all keys while the prompt is up
    }

    if (e == ftxui::Event::Character('q') || e == ftxui::Event::Escape) {
      screen.Exit();
      return true;
    }

    // Map a key to a command. Dangerous ones arm the confirm prompt; the rest
    // issue immediately.
    std::optional<Cmd> chosen;
    if (e == ftxui::Event::Character('a')) chosen = Cmd::Arm;
    else if (e == ftxui::Event::Character('d')) chosen = Cmd::Disarm;
    else if (e == ftxui::Event::Character('t')) chosen = Cmd::Takeoff;
    else if (e == ftxui::Event::Character('l')) chosen = Cmd::Land;
    else if (e == ftxui::Event::Character('r')) chosen = Cmd::Rtl;

    if (!chosen) return false;
    if (needs_confirm(*chosen)) {
      pending_confirm = chosen;
    } else {
      issue(*chosen);
    }
    return true;
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
