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
    default: return "COMMAND";
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
  Element header =
      hbox({text("mavlink-gs") | bold, filler(), arm, text(" "), link});

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

  // --- Command bar: confirmation prompt, or key hints + last status -------
  Element command_bar;
  if (!confirm_prompt.empty()) {
    command_bar = text(" " + confirm_prompt + " ") | bold |
                  color(Color::Black) | bgcolor(Color::Yellow);
  } else {
    Element right = !note.empty() ? text(note) | color(Color::Red)
                                  : command_status(s);
    command_bar = hbox({
        text("[a]rm [d]isarm [t]akeoff [l]and [r]tl  [q]uit") | dim,
        filler(),
        right,
    });
  }

  return vbox({
      header,
      separator(),
      hbox({attitude | flex, position | flex}),
      power,
      separator(),
      command_bar,
  }) | border;
}

}  // namespace mgs
