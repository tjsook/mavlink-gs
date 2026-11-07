#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mgs {

// What one mission step does. These map 1:1 onto MAV_CMD_NAV_* commands.
enum class MissionItemKind { Takeoff, Waypoint, Land, Rtl };

// One step of a mission, as a human wrote it in YAML.
//
// Altitudes are metres above home (MAV_FRAME_GLOBAL_RELATIVE_ALT), because that
// is how people think about a flight ("climb to 20 m"), not metres above sea
// level. Takeoff and land may leave lat/lon out; resolve_positions() fills them
// in before upload. Waypoints must always say where they are.
struct MissionItem {
  MissionItemKind kind = MissionItemKind::Waypoint;
  std::optional<double> lat_deg;
  std::optional<double> lon_deg;
  float alt_m = 0.0f;   // above home
  float hold_s = 0.0f;  // waypoint only: loiter this long before moving on
};

using Mission = std::vector<MissionItem>;

// A mission file that cannot be flown. what() names the item and the problem,
// e.g. "item 3 (waypoint): missing 'lat'", so the user can fix the YAML.
class MissionError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Parse and validate mission YAML. Throws MissionError on any problem — a bad
// mission is refused up front, never half-uploaded. Expected shape:
//
//   items:
//     - { type: takeoff, alt: 10 }
//     - { type: waypoint, lat: 47.3980, lon: 8.5460, alt: 20, hold: 2 }
//     - { type: rtl }
Mission parse_mission(const std::string& yaml);

// Read a file and parse_mission() it. Throws MissionError (also if unreadable).
Mission load_mission(const std::filesystem::path& path);

// Fill in the positions the YAML left out, so every item except RTL has a
// lat/lon: a takeoff without one uses the vehicle's position `here`; a land
// without one uses the previous item's position (or `here` if none).
Mission resolve_positions(Mission mission, double here_lat_deg,
                          double here_lon_deg);

}  // namespace mgs
