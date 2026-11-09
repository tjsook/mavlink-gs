#include "ui/map_panel.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include <ftxui/dom/canvas.hpp>
#include <ftxui/screen/color.hpp>
#include <spdlog/fmt/fmt.h>

namespace mgs {

using namespace ftxui;

namespace {

// Dots kept clear on every side: room for a step number beside the outermost
// waypoint, and for the scale bar along the bottom.
constexpr int kMarginDots = 6;

// The arrow nearest the vehicle's heading. MAVLink yaw is degrees clockwise
// from north, so 0 is up and 90 is right, as on the map.
const char* heading_arrow(double yaw_deg) {
  static constexpr const char* kArrows[] = {"↑", "↗", "→", "↘",
                                            "↓", "↙", "←", "↖"};
  const int sector = static_cast<int>(std::lround(yaw_deg / 45.0));
  return kArrows[((sector % 8) + 8) % 8];
}

std::optional<GeoPoint> position_of(const VehicleMissionItem& item) {
  if (!item.lat_deg || !item.lon_deg) return std::nullopt;
  return GeoPoint{*item.lat_deg, *item.lon_deg};
}

}  // namespace

Element render_map(const VehicleState& s, const std::vector<GeoPoint>& trail) {
  // Everything the view must hold. The first point is the projection origin;
  // any nearby point works, since the fit re-centres on the whole set.
  std::vector<GeoPoint> all;
  if (s.has_home) all.push_back({s.home_lat_deg, s.home_lon_deg});
  if (s.has_position) all.push_back({s.latitude_deg, s.longitude_deg});
  for (const auto& item : s.vehicle_mission) {
    if (const auto at = position_of(item)) all.push_back(*at);
  }
  all.insert(all.end(), trail.begin(), trail.end());
  if (all.empty()) return text("awaiting GPS fix") | dim;

  // The canvas is drawn later, at its final size, so the lambda keeps copies.
  return canvas([s, trail, all](Canvas& c) {
    const GeoPoint origin = all.front();
    std::vector<LocalPoint> local;
    local.reserve(all.size());
    for (const auto& g : all) local.push_back(to_local(g, origin));
    const MapFit fit(local, c.width(), c.height(), kMarginDots);
    const auto dot = [&](GeoPoint g) {
      const LocalPoint p = to_local(g, origin);
      return std::pair{fit.x(p), fit.y(p)};
    };
    const std::optional<GeoPoint> home =
        s.has_home ? std::optional{GeoPoint{s.home_lat_deg, s.home_lon_deg}}
                   : std::nullopt;

    // Back to front: trail, path, steps, home, then the vehicle on top.
    for (const auto& g : trail) {
      const auto [x, y] = dot(g);
      c.DrawPoint(x, y, true, Color::Blue);
    }

    // The path joins each step that has a place to the one before it. RTL has
    // none of its own; it flies home, so its leg ends there.
    std::optional<GeoPoint> prev;
    for (const auto& item : s.vehicle_mission) {
      std::optional<GeoPoint> next = position_of(item);
      if (!next && item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) next = home;
      if (!next) continue;
      if (prev) {
        const auto [x1, y1] = dot(*prev);
        const auto [x2, y2] = dot(*next);
        c.DrawPointLine(x1, y1, x2, y2, Color::GrayDark);
      }
      prev = next;
    }

    // Numbered like the Mission panel; the step being flown in yellow.
    for (std::size_t i = 0; i < s.vehicle_mission.size(); ++i) {
      const auto at = position_of(s.vehicle_mission[i]);
      if (!at) continue;
      const bool current = static_cast<int>(i) == s.mission_current;
      const Color color = current ? Color::Yellow : Color::White;
      const auto [x, y] = dot(*at);
      c.DrawPointCircleFilled(x, y, 1, color);
      c.DrawText(x + 4, y, std::to_string(i + 1), color);
    }

    if (home) {
      const auto [x, y] = dot(*home);
      c.DrawText(x, y, "H", Color::Magenta);
    }
    if (s.has_position) {
      const auto [x, y] = dot({s.latitude_deg, s.longitude_deg});
      c.DrawText(x, y, heading_arrow(s.yaw_deg), [](Cell& cell) {
        cell.foreground_color = Color::Green;
        cell.bold = true;
      });
    }

    // Scale bar, bottom left: a round length about a quarter of the width.
    const double mpd = fit.metres_per_dot();
    const double len = nice_scale_length(mpd * c.width() / 4.0);
    const int bar = static_cast<int>(std::lround(len / mpd));
    const int y = c.height() - 2;
    c.DrawPointLine(0, y, bar, y);
    c.DrawPointLine(0, y - 2, 0, y + 1);
    c.DrawPointLine(bar, y - 2, bar, y + 1);
    c.DrawText(bar + 4, c.height() - 1, fmt::format("{:g} m", len));
    c.DrawText(c.width() - 2, 0, "N");
  });
}

}  // namespace mgs
