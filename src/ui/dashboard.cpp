#include "ui/dashboard.hpp"

#include <string>
#include <utility>

#include <ftxui/screen/color.hpp>
#include <spdlog/fmt/fmt.h>

#include "ui/map_panel.hpp"

namespace mgs {

using namespace ftxui;

namespace {

// Width of the left column (Attitude, Position, Power). The map takes the
// rest of the row, so it gets whatever a wider terminal adds.
constexpr int kLeftColumnWidth = 32;

// Most mission steps the panel lists at once. A longer mission scrolls so the
// step being flown stays in view.
constexpr int kMaxMissionRows = 6;

// One "Label ............ value" row: label dim on the left, value bold on the
// right, filler() pushing them apart.
Element field(const std::string& label, const std::string& value) {
  return hbox({text(label) | dim, filler(), text(value) | bold});
}

// A titled, bordered box around a stack of rows.
Element panel(const std::string& title, Elements rows) {
  return window(text(" " + title + " "), vbox(std::move(rows)));
}

// Short label for the MAV_CMD id we most recently sent.
std::string command_label(std::uint16_t command) {
  switch (command) {
    case MAV_CMD_COMPONENT_ARM_DISARM: return "ARM/DISARM";
    case MAV_CMD_NAV_TAKEOFF: return "TAKEOFF";
    case MAV_CMD_NAV_LAND: return "LAND";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH: return "RTL";
    case MAV_CMD_MISSION_START: return "MISSION START";
    default: return "COMMAND";
  }
}

// Header chip for the flight-log recorder: a red "REC" dot while healthy, a
// loud warning if a write failed mid-session, nothing when logging is off.
Element log_indicator(const VehicleState& s) {
  switch (s.log_status) {
    case LogStatus::Recording:
      return text(" ● REC ") | color(Color::Red) | bold;
    case LogStatus::Failed:
      return text(" LOG FAIL ") | color(Color::White) | bgcolor(Color::Red);
    case LogStatus::Off:
    default:
      return text("");
  }
}

// Why the vehicle refused a mission upload (MAV_MISSION_RESULT), in words.
std::string mission_error_label(std::uint8_t result) {
  switch (result) {
    case MAV_MISSION_UNSUPPORTED_FRAME: return "unsupported frame";
    case MAV_MISSION_UNSUPPORTED: return "unsupported command";
    case MAV_MISSION_NO_SPACE: return "too many items";
    case MAV_MISSION_INVALID:
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7: return "invalid item";
    case MAV_MISSION_INVALID_SEQUENCE: return "bad sequence";
    case MAV_MISSION_DENIED: return "denied";
    default: return fmt::format("error {}", result);
  }
}

// Short name for a downloaded step's MAV_CMD. Any command we do not name
// (another GCS may have uploaded it) still gets a row, by number.
std::string mission_command_label(std::uint16_t command) {
  switch (command) {
    case MAV_CMD_NAV_TAKEOFF: return "TAKEOFF";
    case MAV_CMD_NAV_WAYPOINT: return "WAYPOINT";
    case MAV_CMD_NAV_LAND: return "LAND";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH: return "RTL";
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TO_ALT: return "LOITER";
    case MAV_CMD_DO_CHANGE_SPEED: return "SPEED";
    default: return fmt::format("CMD {}", command);
  }
}

// A step's target altitude, if it has one, marked by what it is measured
// from: plain metres are above home, the common case.
std::string mission_alt_label(const VehicleMissionItem& item) {
  const bool has_alt = item.command == MAV_CMD_NAV_TAKEOFF ||
                       (item.lat_deg && item.command != MAV_CMD_NAV_LAND);
  if (!has_alt) return "";
  switch (item.frame) {
    case MAV_FRAME_GLOBAL:
    case MAV_FRAME_GLOBAL_INT: return fmt::format("{:.0f} m MSL", item.alt_m);
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
      return fmt::format("{:.0f} m AGL", item.alt_m);
    default: return fmt::format("{:.0f} m", item.alt_m);
  }
}

// The steps the vehicle holds, numbered from 1 like the YAML. The one it is
// flying is marked and focused, so the frame scrolls to keep it in view.
Element mission_list(const VehicleState& s) {
  Elements rows;
  for (std::size_t i = 0; i < s.vehicle_mission.size(); ++i) {
    const auto& item = s.vehicle_mission[i];
    const bool current = static_cast<int>(i) == s.mission_current;
    Element row = hbox({
        text(current ? "▶" : " "),
        text(fmt::format("{:>3} ", i + 1)) | dim,
        text(mission_command_label(item.command)),
        filler(),
        text(mission_alt_label(item)),
    });
    rows.push_back(current ? row | bold | color(Color::Cyan) | focus : row);
  }
  return vbox(std::move(rows)) | vscroll_indicator | yframe |
         size(HEIGHT, LESS_THAN, kMaxMissionRows);
}

// The mission the vehicle holds: fetch progress, or the list once we have it.
Elements download_rows(const VehicleState& s) {
  switch (s.download_status) {
    case DownloadStatus::Downloading: {
      const float done =
          s.download_count ? static_cast<float>(s.download_received) /
                                 static_cast<float>(s.download_count)
                           : 0.0f;
      return {field("Fetch", fmt::format("{}/{}", s.download_received,
                                         s.download_count)) |
                  color(Color::Yellow),
              gauge(done) | color(Color::Yellow)};
    }
    case DownloadStatus::Downloaded:
      if (s.vehicle_mission.empty()) return {text("vehicle has no mission") | dim};
      return {mission_list(s)};
    case DownloadStatus::TimedOut:
      return {field("Fetch", "timed out") | color(Color::Red)};
    case DownloadStatus::None:
    default:
      return {};
  }
}

// The upload status line: progress while uploading, then the outcome.
Elements upload_rows(const VehicleState& s) {
  const auto progress = fmt::format("{}/{}", s.mission_sent, s.mission_count);
  switch (s.mission_status) {
    case MissionStatus::Uploading: {
      const float done =
          s.mission_count ? static_cast<float>(s.mission_sent) /
                                static_cast<float>(s.mission_count)
                          : 0.0f;
      return {field("Upload", progress) | color(Color::Yellow),
              gauge(done) | color(Color::Yellow)};
    }
    case MissionStatus::Uploaded:
      return {field("Upload", fmt::format("{} items", s.mission_count)) |
              color(Color::Green)};
    case MissionStatus::Rejected: {
      // mission_sent is the last item the vehicle asked for — the one it then
      // refused — and, being 1-based, matches the item number in the YAML.
      const auto why = mission_error_label(s.mission_error);
      return {field("Rejected", s.mission_sent
                                    ? fmt::format("item {}: {}",
                                                  s.mission_sent, why)
                                    : why) |
              color(Color::Red)};
    }
    case MissionStatus::TimedOut:
      return {field("Timed out", progress) | color(Color::Red)};
    case MissionStatus::None:
    default:
      return {};
  }
}

// The Mission panel body: the last upload's status over the vehicle's mission.
Elements mission_rows(const VehicleState& s) {
  Elements rows = upload_rows(s);
  for (auto& row : download_rows(s)) rows.push_back(std::move(row));
  if (rows.empty()) rows.push_back(text("no mission yet") | dim);
  return rows;
}

// The right-hand status chip: outcome of the last command, colored by result.
Element command_status(const VehicleState& s) {
  switch (s.last_command_status) {
    case CommandStatus::Pending:
      return text(command_label(s.last_command_id) + " pending…") |
             color(Color::Yellow);
    case CommandStatus::Accepted:
      return text(command_label(s.last_command_id) + " accepted") |
             color(Color::Green);
    case CommandStatus::Rejected:
      return text(command_label(s.last_command_id) + " rejected") |
             color(Color::Red);
    case CommandStatus::TimedOut:
      return text(command_label(s.last_command_id) + " timed out") |
             color(Color::Red);
    case CommandStatus::None:
    default:
      return text("");
  }
}

}  // namespace

Element render_dashboard(const VehicleState& s, bool connected,
                         const std::string& confirm_prompt,
                         const std::string& note,
                         const std::vector<GeoPoint>& trail) {
  // --- Header: program name, arm state, link state ------------------------
  Element link = connected
      ? text(" CONNECTED ") | color(Color::Black) | bgcolor(Color::Green)
      : text(" DISCONNECTED ") | color(Color::White) | bgcolor(Color::Red);
  Element arm = s.armed
      ? text(" ARMED ") | color(Color::Black) | bgcolor(Color::Yellow)
      : text(" DISARMED ") | dim;
  Element header = hbox({text("mavlink-gs") | bold, filler(),
                         log_indicator(s), arm, text(" "), link});

  // --- Attitude -----------------------------------------------------------
  Element attitude = panel("Attitude", {
      field("Roll", fmt::format("{:+.1f}°", s.roll_deg)),
      field("Pitch", fmt::format("{:+.1f}°", s.pitch_deg)),
      field("Yaw", fmt::format("{:+.1f}°", s.yaw_deg)),
  });

  // --- Position (dashes until the first GPS fix) --------------------------
  Elements position_rows =
      s.has_position
          ? Elements{
                field("Lat", fmt::format("{:.7f}°", s.latitude_deg)),
                field("Lon", fmt::format("{:.7f}°", s.longitude_deg)),
                field("Alt (MSL)", fmt::format("{:.1f} m", s.altitude_m)),
                field("Rel alt", fmt::format("{:.1f} m", s.relative_alt_m)),
            }
          : Elements{text("awaiting GPS fix") | dim};
  Element position = panel("Position", std::move(position_rows));

  // --- Power (gauge only once a battery percentage is known) --------------
  Elements power_rows{
      field("Voltage", fmt::format("{:.2f} V", s.battery_volts)),
      field("Current", fmt::format("{:.2f} A", s.battery_amps)),
  };
  if (s.battery_remaining >= 0) {
    power_rows.push_back(hbox({
        text("Charge ") | dim,
        gauge(static_cast<float>(s.battery_remaining) / 100.0f) | flex,
        text(fmt::format(" {}%", s.battery_remaining)),
    }));
  } else {
    power_rows.push_back(field("Charge", "--"));
  }
  Element power = panel("Power", std::move(power_rows));
  Element mission = panel("Mission", mission_rows(s));
  Element map = panel("Map", {render_map(s, trail) | flex});

  // --- Command bar: key hints over the last outcome (or a note) ----------
  // Two lines, not one: the hints alone take most of an 80-column terminal, so
  // a long status ("MISSION START accepted") beside them would be cut off. A
  // confirmation prompt replaces the hints while it waits for y/n.
  Element hints =
      !confirm_prompt.empty()
          ? text(" " + confirm_prompt + " ") | bold | color(Color::Black) |
                bgcolor(Color::Yellow)
          : text("[a]rm [d]isarm [t]akeoff [l]and [r]tl [u]pload [s]tart "
                 "[f]etch [q]uit") |
                dim;
  Element status = !note.empty() ? text(note) | color(Color::Red)
                                 : command_status(s);
  Element command_bar = vbox({hbox({hints, filler()}), hbox({status, filler()})});

  // Telemetry down the left; the map over the mission list on the right. The
  // body row is flex, so on a tall terminal the map grows and the command bar
  // stays at the bottom.
  Element left = vbox({attitude, position, power}) |
                 size(WIDTH, EQUAL, kLeftColumnWidth);
  Element right = vbox({map | flex, mission});
  return vbox({
      header,
      separator(),
      hbox({left, right | flex}) | flex,
      separator(),
      command_bar,
  }) | border;
}

}  // namespace mgs
