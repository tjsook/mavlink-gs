#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "mission/mission.hpp"

namespace {

using Catch::Matchers::ContainsSubstring;
using mgs::MissionItemKind;

const char* kGood = R"(
items:
  - { type: takeoff, alt: 10 }
  - { type: waypoint, lat: 47.3980, lon: 8.5460, alt: 20, hold: 2 }
  - { type: land }
  - { type: rtl }
)";

// Parse `yaml` and return the MissionError text (empty if it parsed).
std::string error_of(const std::string& yaml) {
  try {
    mgs::parse_mission(yaml);
  } catch (const mgs::MissionError& e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST_CASE("a well-formed mission parses into typed items", "[mission]") {
  const auto m = mgs::parse_mission(kGood);
  REQUIRE(m.size() == 4);

  CHECK(m[0].kind == MissionItemKind::Takeoff);
  CHECK(m[0].alt_m == 10.0f);
  CHECK_FALSE(m[0].lat_deg.has_value());  // filled in later

  CHECK(m[1].kind == MissionItemKind::Waypoint);
  CHECK(*m[1].lat_deg == 47.3980);
  CHECK(*m[1].lon_deg == 8.5460);
  CHECK(m[1].alt_m == 20.0f);
  CHECK(m[1].hold_s == 2.0f);

  CHECK(m[2].kind == MissionItemKind::Land);
  CHECK(m[3].kind == MissionItemKind::Rtl);
}

TEST_CASE("bad missions are refused with a pointed message", "[mission]") {
  CHECK_THAT(error_of("{ not: [valid"), ContainsSubstring("invalid YAML"));
  CHECK_THAT(error_of("waypoints: []"), ContainsSubstring("'items:'"));
  CHECK_THAT(error_of("items: []"), ContainsSubstring("empty"));
  CHECK_THAT(error_of("items: [ { type: hover } ]"),
             ContainsSubstring("unknown type 'hover'"));
  CHECK_THAT(error_of("items: [ { alt: 5 } ]"),
             ContainsSubstring("missing 'type'"));
  CHECK_THAT(error_of("items: [ { type: waypoint, alt: 5 } ]"),
             ContainsSubstring("item 1 (waypoint): missing 'lat'"));
  CHECK_THAT(error_of("items: [ { type: waypoint, lat: 1, lon: 2 } ]"),
             ContainsSubstring("missing 'alt'"));
  CHECK_THAT(error_of("items: [ { type: takeoff, altitude: 5 } ]"),
             ContainsSubstring("unknown field 'altitude'"));
  CHECK_THAT(error_of("items: [ { type: waypoint, lat: 1, alt: 5 } ]"),
             ContainsSubstring("both 'lat' and 'lon'"));
  CHECK_THAT(error_of("items: [ { type: waypoint, lat: 95, lon: 2, alt: 5 } ]"),
             ContainsSubstring("[-90, 90]"));
  CHECK_THAT(error_of("items: [ { type: takeoff, alt: high } ]"),
             ContainsSubstring("'alt' is not a number"));
  CHECK_THAT(error_of("items: [ { type: takeoff, alt: 0 } ]"),
             ContainsSubstring("above 0"));
  // The item number points at the broken one, not the first.
  CHECK_THAT(error_of("items: [ { type: rtl }, { type: rtl, alt: 3 } ]"),
             ContainsSubstring("item 2"));
}

TEST_CASE("load_mission reports an unreadable file", "[mission]") {
  CHECK_THROWS_AS(mgs::load_mission("/no/such/mission.yaml"),
                  mgs::MissionError);
}

TEST_CASE("resolve_positions fills takeoff and land positions", "[mission]") {
  const auto m = mgs::resolve_positions(mgs::parse_mission(kGood),
                                        /*here_lat=*/47.0, /*here_lon=*/8.0);

  // Takeoff lifts off where the vehicle is.
  CHECK(*m[0].lat_deg == 47.0);
  CHECK(*m[0].lon_deg == 8.0);
  // Land comes down where the previous step (the waypoint) ended.
  CHECK(*m[2].lat_deg == 47.3980);
  CHECK(*m[2].lon_deg == 8.5460);
  // RTL stays positionless.
  CHECK_FALSE(m[3].lat_deg.has_value());
}
