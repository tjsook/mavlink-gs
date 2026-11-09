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
#include "map/trail.hpp"
#include "mavlink/command_tracker.hpp"
#include "mavlink/commands.hpp"
#include "mavlink/decoder.hpp"
#include "mavlink/mission_downloader.hpp"
#include "mavlink/mission_uploader.hpp"
#include "mission/mission.hpp"
#include "net/command_queue.hpp"
#include "net/udp_link.hpp"
#include "net/waker.hpp"
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

// How often we announce ourselves to the vehicle with a GCS heartbeat. Without
// this the autopilot reports "no connection to the GCS" and refuses to arm.
constexpr auto kHeartbeatPeriod = 1s;

// Height above the vehicle's current altitude to request on takeoff.
constexpr float kTakeoffHeightM = 2.5f;

// The commands the palette can issue.
enum class Cmd { Arm, Disarm, Takeoff, Land, Rtl, Upload, Start, Fetch };

// Dangerous commands get a y/n confirmation; the rest fire immediately. Upload
// only stores the mission on the vehicle; Start is what makes it fly.
bool needs_confirm(Cmd c) {
  return c == Cmd::Arm || c == Cmd::Takeoff || c == Cmd::Rtl ||
         c == Cmd::Start;
}

const char* cmd_name(Cmd c) {
  switch (c) {
    case Cmd::Arm: return "ARM";
    case Cmd::Disarm: return "DISARM";
    case Cmd::Takeoff: return "TAKEOFF";
    case Cmd::Land: return "LAND";
    case Cmd::Rtl: return "RTL";
    case Cmd::Upload: return "UPLOAD MISSION";
    case Cmd::Start: return "START MISSION";
    case Cmd::Fetch: return "FETCH MISSION";
  }
  return "";
}

// A mission the UI hands to the network thread to upload: positions already
// resolved against the vehicle, and the vehicle's address to send it to.
struct UploadRequest {
  mgs::Mission mission;
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
};

// A request to read back the mission the vehicle holds, and where to ask.
struct FetchRequest {
  std::uint8_t target_system = 0;
  std::uint8_t target_component = 0;
};

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

  std::string mission_path;
  app.add_option("--mission", mission_path,
                 "YAML mission file to upload with the [u] key");

  CLI11_PARSE(app, argc, argv);

  // Load and validate the mission now, before the TUI, so a typo in the YAML is
  // reported plainly and we exit — not discovered mid-flight on a keypress.
  std::optional<mgs::Mission> mission;
  if (!mission_path.empty()) {
    try {
      mission = mgs::load_mission(mission_path);
    } catch (const mgs::MissionError& e) {
      spdlog::error("bad mission: {}", e.what());
      return 1;
    }
    spdlog::info("loaded mission {} ({} items)", mission_path, mission->size());
  }

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
  mgs::Waker wake;  // knocks the network thread out of poll()
  mgs::CommandQueue commands(wake);  // UI thread pushes; network thread sends
  mgs::HandoffQueue<UploadRequest> uploads(wake);  // same, for whole missions
  mgs::HandoffQueue<FetchRequest> fetches(wake);   // ...and mission read-backs
  std::atomic<bool> stop{false};
  std::string net_error;  // read only after network.join() (see below)

  // --- Network thread: receive -> decode -> fold into shared state --------
  // Owns the socket (moved in). receive() sleeps until a datagram arrives, the
  // Waker rings (a queued command, or shutdown), or its poll timeout ends.
  std::thread network([&, link = std::move(link),
                       log = std::move(flight_log)]() mutable {
    mgs::MavlinkDecoder decoder;
    mgs::CommandTracker tracker;     // one in-flight command at a time
    mavlink_message_t in_flight{};   // the message to resend on retry
    mgs::CommandStatus published = mgs::CommandStatus::None;
    mgs::MissionUploader uploader;   // one mission upload at a time
    mgs::MissionStatus mission_published = mgs::MissionStatus::None;
    std::uint16_t mission_sent_published = 0;
    mgs::MissionDownloader downloader;  // one mission download at a time
    mgs::DownloadStatus download_published = mgs::DownloadStatus::None;
    std::uint16_t download_received_published = 0;
    // The vehicle runs one mission transfer at a time (a second one is refused
    // as "busy"), so requests wait here until the current transfer ends.
    std::optional<UploadRequest> pending_upload;
    std::optional<FetchRequest> pending_fetch;
    bool fetched_on_connect = false;
    mgs::LogStatus log_published = mgs::LogStatus::Off;
    auto last_flush = std::chrono::steady_clock::now();
    // Back-date so the first heartbeat goes out as soon as we know the vehicle.
    auto last_heartbeat_sent =
        std::chrono::steady_clock::now() - kHeartbeatPeriod;

    // The log timestamp is wall-clock (system_clock), not the steady_clock the
    // loop uses for timeouts: a .tlog means nothing to QGroundControl unless its
    // times are real "since 1970" times. Two clocks, two jobs.
    const auto log_now = [] { return std::chrono::system_clock::now(); };

    // Every outgoing message goes through here: send it, and record it in the
    // flight log if the send worked. Returns whether it was sent.
    const auto send = [&](const mavlink_message_t& msg) {
      if (!link->send(msg)) return false;
      if (log) log->record(msg, log_now());
      return true;
    };

    try {
      while (!stop) {
        const auto now = std::chrono::steady_clock::now();

        // receive() returns nullopt on a timeout or a wake. We do not
        // `continue` then: command handling below must run every pass, not
        // only when a datagram happened to arrive.
        if (const auto packet = link->receive(wake.fd())) {
          for (const auto& msg : decoder.parse(*packet)) {
            shared.modify(
                [&](mgs::VehicleState& s) { mgs::update(s, msg, now); });
            if (log) log->record(msg, log_now());  // record what we received
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
              mavlink_command_ack_t ack;
              mavlink_msg_command_ack_decode(&msg, &ack);
              tracker.on_ack(ack);  // resolve/accept/reject the in-flight cmd
            }
            // Mid-upload, the vehicle asks for items one by one; answer each.
            if (const auto reply = uploader.on_message(msg, now)) send(*reply);
            // Mid-download, each item it sends earns a request for the next.
            if (const auto reply = downloader.on_message(msg, now)) send(*reply);
            // Read the vehicle's mission as soon as we know who it is, so the
            // panel shows what it will fly before anyone uploads anything.
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && !fetched_on_connect) {
              pending_fetch = FetchRequest{msg.sysid, msg.compid};
              fetched_on_connect = true;
            }
          }
        }

        // Clear the wake BEFORE draining. A push that lands between the two
        // leaves a fresh byte in the pipe, so the worst case is one harmless
        // extra wake. The reverse order could erase that push's byte after we
        // took the queue, stranding its command until the next poll timeout.
        wake.clear();

        // Send commands the UI queued since the last pass. Drained here, on the
        // socket-owning thread, so the UI thread never touches the socket.
        for (const auto& cmd : commands.drain()) {
          if (send(cmd)) {
            mavlink_command_long_t cl;
            mavlink_msg_command_long_decode(&cmd, &cl);
            tracker.begin(cl.command, now);  // count this send as attempt 1
            in_flight = cmd;
          }
        }

        // Mission transfers the UI asked for. If it asked twice before we got
        // to them, the latest request wins. An upload changes what the vehicle
        // holds, so each one also queues a fetch to read the result back: the
        // panel then shows the vehicle's copy, not ours.
        for (auto& up : uploads.drain()) {
          pending_fetch = FetchRequest{up.target_system, up.target_component};
          pending_upload = std::move(up);
        }
        for (const auto& f : fetches.drain()) pending_fetch = f;

        // Start the next transfer once none is running; uploads go first. The
        // first message only opens it — the rest is driven from on_message().
        const bool transfer_busy =
            uploader.status() == mgs::MissionStatus::Uploading ||
            downloader.status() == mgs::DownloadStatus::Downloading;
        if (!transfer_busy && pending_upload) {
          send(uploader.begin(pending_upload->mission,
                              pending_upload->target_system,
                              pending_upload->target_component, now));
          pending_upload.reset();
        } else if (!transfer_busy && pending_fetch) {
          send(downloader.begin(pending_fetch->target_system,
                                pending_fetch->target_component, now));
          pending_fetch.reset();
        }

        // Retransmit the in-flight command, or the stalled transfer step, if
        // the vehicle has gone quiet on it.
        if (tracker.should_resend(now)) send(in_flight);
        if (const auto again = uploader.retransmit(now)) send(*again);
        if (const auto again = downloader.retransmit(now)) send(*again);

        // Announce ourselves as a GCS about once a second. send() is a no-op
        // until the first datagram tells us the vehicle's address, so these
        // start flowing as soon as telemetry arrives.
        if (now - last_heartbeat_sent >= kHeartbeatPeriod) {
          send(mgs::gcs_heartbeat());
          last_heartbeat_sent = now;
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

        // Publish mission upload progress only when it changes.
        if (uploader.status() != mission_published ||
            uploader.sent() != mission_sent_published) {
          mission_published = uploader.status();
          mission_sent_published = uploader.sent();
          shared.modify([&](mgs::VehicleState& s) {
            s.mission_status = uploader.status();
            s.mission_count = uploader.count();
            s.mission_sent = uploader.sent();
            s.mission_error = uploader.error();
          });
        }

        // Publish download progress only when it changes. The item list is
        // copied once, when complete — not on every item.
        if (downloader.status() != download_published ||
            downloader.received() != download_received_published) {
          download_published = downloader.status();
          download_received_published = downloader.received();
          shared.modify([&](mgs::VehicleState& s) {
            s.download_status = downloader.status();
            s.download_count = downloader.count();
            s.download_received = downloader.received();
            if (downloader.status() == mgs::DownloadStatus::Downloaded) {
              s.vehicle_mission = downloader.items();
            }
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
  mgs::Trail trail;                    // recent positions, for the map

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
      case Cmd::Upload:
        if (!mission) {
          note = "no mission loaded — start with --mission FILE";
          return;
        }
        // Takeoff/land items may lack a position; they need ours to fill in.
        if (!snap.has_position) {
          note = "no position fix — cannot place the mission";
          return;
        }
        uploads.push({mgs::resolve_positions(*mission, snap.latitude_deg,
                                             snap.longitude_deg),
                      ts, tc});
        break;
      case Cmd::Start:
        commands.push(mgs::mission_start_command(ts, tc));
        break;
      case Cmd::Fetch:
        fetches.push({ts, tc});
        break;
    }
    note.clear();
  };

  auto view = ftxui::Renderer([&] {
    const mgs::VehicleState snap = shared.snapshot();
    // Sample the trail from the snapshot we already took: the network thread
    // needs no part in it. Trail skips samples where the vehicle sat still.
    if (snap.has_position) {
      trail.add({snap.latitude_deg, snap.longitude_deg},
                std::chrono::steady_clock::now());
    }
    const bool connected =
        snap.last_heartbeat != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - snap.last_heartbeat <
            kHeartbeatTimeout;
    const std::string prompt =
        pending_confirm
            ? std::string(cmd_name(*pending_confirm)) + "? confirm (y/n)"
            : "";
    return mgs::render_dashboard(snap, connected, prompt, note, trail.points());
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
    else if (e == ftxui::Event::Character('u')) chosen = Cmd::Upload;
    else if (e == ftxui::Event::Character('s')) chosen = Cmd::Start;
    else if (e == ftxui::Event::Character('f')) chosen = Cmd::Fetch;

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
  wake.notify();  // don't make shutdown wait out the poll timeout
  ticker.join();
  network.join();
  if (!net_error.empty()) {
    spdlog::error("network link stopped: {}", net_error);
  }
  spdlog::info("mavlink-gs shut down cleanly");
  return 0;
}
