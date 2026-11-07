#include "ui/dashboard.hpp"

#include <string>
#include <utility>

#include <ftxui/screen/color.hpp>
#include <spdlog/fmt/fmt.h>

namespace mgs {

using namespace ftxui;

namespace {

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

// The Mission panel body: one status line, plus a gauge while uploading.
Elements mission_rows(const VehicleState& s) {
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
    case MissionStatus::Rejected:
      return {field("Rejected", mission_error_label(s.mission_error)) |
              color(Color::Red)};
    case MissionStatus::TimedOut:
      return {field("Timed out", progress) | color(Color::Red)};
    case MissionStatus::None:
    default:
      return {text("no mission uploaded") | dim};
  }
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
                         const std::string& note) {
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

  // --- Command bar: key hints over the last outcome (or a note) ----------
  // Two lines, not one: the hints alone take most of an 80-column terminal, so
  // a long status ("MISSION START accepted") beside them would be cut off. A
  // confirmation prompt replaces the hints while it waits for y/n.
  Element hints =
      !confirm_prompt.empty()
          ? text(" " + confirm_prompt + " ") | bold | color(Color::Black) |
                bgcolor(Color::Yellow)
          : text("[a]rm [d]isarm [t]akeoff [l]and [r]tl [u]pload [s]tart "
                 "[q]uit") |
                dim;
  Element status = !note.empty() ? text(note) | color(Color::Red)
                                 : command_status(s);
  Element command_bar = vbox({hbox({hints, filler()}), hbox({status, filler()})});

  return vbox({
      header,
      separator(),
      hbox({attitude | flex, position | flex}),
      hbox({power | flex, mission | flex}),
      separator(),
      command_bar,
  }) | border;
}

}  // namespace mgs
