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

}  // namespace

Element render_dashboard(const VehicleState& s, bool connected) {
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

  return vbox({
      header,
      separator(),
      hbox({attitude | flex, position | flex}),
      power,
  }) | border;
}

}  // namespace mgs
