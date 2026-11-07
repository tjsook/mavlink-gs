#include "mission/mission.hpp"

#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string_view>

#include <spdlog/fmt/fmt.h>
#include <yaml-cpp/yaml.h>

namespace mgs {
namespace {

// The YAML spelling of each kind, and the keys each kind may carry. Anything
// else is an error: a typo like "altitude" must not be silently ignored.
struct KindSpec {
  std::string_view name;
  MissionItemKind kind;
  std::set<std::string_view> keys;
};

const KindSpec kKinds[] = {
    {"takeoff", MissionItemKind::Takeoff, {"type", "lat", "lon", "alt"}},
    {"waypoint", MissionItemKind::Waypoint,
     {"type", "lat", "lon", "alt", "hold"}},
    {"land", MissionItemKind::Land, {"type", "lat", "lon"}},
    {"rtl", MissionItemKind::Rtl, {"type"}},
};

// Read one numeric field, turning yaml-cpp's conversion error into ours.
double number(const YAML::Node& node, const std::string& key,
              const std::string& where) {
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception&) {
    throw MissionError(fmt::format("{}: '{}' is not a number", where, key));
  }
}

MissionItem parse_item(const YAML::Node& node, std::size_t index) {
  const std::string at = fmt::format("item {}", index + 1);  // 1-based for humans
  if (!node.IsMap()) throw MissionError(at + ": expected a map of fields");
  if (!node["type"]) throw MissionError(at + ": missing 'type'");

  const auto type = node["type"].as<std::string>();
  const KindSpec* spec = nullptr;
  for (const auto& k : kKinds) {
    if (k.name == type) spec = &k;
  }
  if (!spec) {
    throw MissionError(fmt::format(
        "{}: unknown type '{}' (takeoff, waypoint, land, rtl)", at, type));
  }
  const std::string where = fmt::format("{} ({})", at, type);

  for (const auto& kv : node) {
    const auto key = kv.first.as<std::string>();
    if (!spec->keys.contains(key)) {
      throw MissionError(fmt::format("{}: unknown field '{}'", where, key));
    }
  }

  MissionItem item;
  item.kind = spec->kind;

  // lat/lon travel together: one without the other is a typo, not a choice.
  if (node["lat"].IsDefined() != node["lon"].IsDefined()) {
    throw MissionError(where + ": give both 'lat' and 'lon', or neither");
  }
  if (node["lat"]) {
    item.lat_deg = number(node, "lat", where);
    item.lon_deg = number(node, "lon", where);
    if (*item.lat_deg < -90.0 || *item.lat_deg > 90.0) {
      throw MissionError(where + ": 'lat' must be within [-90, 90]");
    }
    if (*item.lon_deg < -180.0 || *item.lon_deg > 180.0) {
      throw MissionError(where + ": 'lon' must be within [-180, 180]");
    }
  } else if (item.kind == MissionItemKind::Waypoint) {
    throw MissionError(where + ": missing 'lat' and 'lon'");
  }

  const bool needs_alt = item.kind == MissionItemKind::Takeoff ||
                         item.kind == MissionItemKind::Waypoint;
  if (needs_alt) {
    if (!node["alt"]) throw MissionError(where + ": missing 'alt'");
    item.alt_m = static_cast<float>(number(node, "alt", where));
    if (item.alt_m <= 0.0f) {
      throw MissionError(where + ": 'alt' must be above 0 (metres above home)");
    }
  }

  if (node["hold"]) {
    item.hold_s = static_cast<float>(number(node, "hold", where));
    if (item.hold_s < 0.0f) throw MissionError(where + ": 'hold' must be >= 0");
  }
  return item;
}

}  // namespace

Mission parse_mission(const std::string& yaml) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception& e) {
    throw MissionError(fmt::format("invalid YAML: {}", e.what()));
  }

  const YAML::Node items = root.IsMap() ? root["items"] : YAML::Node{};
  if (!items || !items.IsSequence()) {
    throw MissionError("expected a top-level 'items:' list");
  }
  if (items.size() == 0) throw MissionError("'items' is empty");
  // MISSION_COUNT carries a uint16, and seq 65535 is reserved.
  if (items.size() >= std::numeric_limits<std::uint16_t>::max()) {
    throw MissionError("too many items for one MAVLink mission");
  }

  Mission mission;
  mission.reserve(items.size());
  for (std::size_t i = 0; i < items.size(); ++i) {
    try {
      mission.push_back(parse_item(items[i], i));
    } catch (const YAML::Exception& e) {  // e.g. a 'type' that is a list
      throw MissionError(fmt::format("item {}: {}", i + 1, e.what()));
    }
  }
  return mission;
}

Mission load_mission(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw MissionError(fmt::format("cannot read {}", path.string()));
  std::ostringstream text;
  text << in.rdbuf();
  try {
    return parse_mission(text.str());
  } catch (const MissionError& e) {
    throw MissionError(fmt::format("{}: {}", path.string(), e.what()));
  }
}

Mission resolve_positions(Mission mission, double here_lat_deg,
                          double here_lon_deg) {
  double last_lat = here_lat_deg;
  double last_lon = here_lon_deg;
  for (auto& item : mission) {
    if (item.kind == MissionItemKind::Rtl) continue;  // RTL has no position
    if (!item.lat_deg) {
      // Takeoff: lift off where the vehicle sits. Land: come down where the
      // previous step ended. Both are what a pilot means by leaving it out.
      const bool takeoff = item.kind == MissionItemKind::Takeoff;
      item.lat_deg = takeoff ? here_lat_deg : last_lat;
      item.lon_deg = takeoff ? here_lon_deg : last_lon;
    }
    last_lat = *item.lat_deg;
    last_lon = *item.lon_deg;
  }
  return mission;
}

}  // namespace mgs
