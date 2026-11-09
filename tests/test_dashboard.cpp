#include <regex>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>

#include "state/vehicle_state.hpp"
#include "ui/dashboard.hpp"

namespace {

// Render an element to a fixed-size character grid and flatten to text so we
// can assert on the content the user would see. ToString() wraps styled runs
// (bold, dim, color) in ANSI escape codes; strip them, or "1 TAKEOFF" would
// not match when the number is dim and the label is not.
std::string render_to_text(const ftxui::Element& element) {
  auto screen = ftxui::Screen::Create(ftxui::Dimension::Fixed(80),
                                      ftxui::Dimension::Fixed(24));
  // Render mutates a copy of the shared element tree; take one so `element`
  // stays const-correct at the call site.
  ftxui::Element copy = element;
  ftxui::Render(screen, copy);
  static const std::regex kAnsi("\x1b\\[[0-9;]*m");
  return std::regex_replace(screen.ToString(), kAnsi, "");
}

}  // namespace

TEST_CASE("dashboard shows DISCONNECTED and awaits a fix by default", "[ui]") {
  const mgs::VehicleState state;  // never heard from a vehicle

  const std::string out = render_to_text(mgs::render_dashboard(state, false));

  CHECK(out.find("DISCONNECTED") != std::string::npos);
  CHECK(out.find("DISARMED") != std::string::npos);
  CHECK(out.find("Attitude") != std::string::npos);
  CHECK(out.find("awaiting GPS fix") != std::string::npos);
}

TEST_CASE("dashboard reflects a live, armed vehicle", "[ui]") {
  mgs::VehicleState state;
  state.armed = true;
  state.has_position = true;
  state.battery_remaining = 42;

  const std::string out = render_to_text(mgs::render_dashboard(state, true));

  CHECK(out.find("CONNECTED") != std::string::npos);
  CHECK(out.find("DISCONNECTED") == std::string::npos);  // not the "dis" state
  CHECK(out.find("ARMED") != std::string::npos);
  CHECK(out.find("42%") != std::string::npos);
  CHECK(out.find("awaiting GPS fix") == std::string::npos);
}

TEST_CASE("dashboard shows the command hint bar by default", "[ui]") {
  const mgs::VehicleState state;
  const std::string out = render_to_text(mgs::render_dashboard(state, false));
  CHECK(out.find("[q]uit") != std::string::npos);     // hint bar present
  CHECK(out.find("[t]akeoff") != std::string::npos);  // bracketed mnemonics
}

TEST_CASE("dashboard shows a confirmation prompt when asked", "[ui]") {
  const mgs::VehicleState state;
  const std::string out = render_to_text(
      mgs::render_dashboard(state, true, "TAKEOFF? confirm (y/n)"));
  CHECK(out.find("TAKEOFF? confirm") != std::string::npos);
}

TEST_CASE("dashboard shows the last command's outcome", "[ui]") {
  mgs::VehicleState state;
  state.last_command_status = mgs::CommandStatus::Rejected;
  state.last_command_id = MAV_CMD_NAV_TAKEOFF;

  const std::string out = render_to_text(mgs::render_dashboard(state, true));
  CHECK(out.find("TAKEOFF rejected") != std::string::npos);
}

TEST_CASE("dashboard shows the recording indicator", "[ui]") {
  mgs::VehicleState state;

  SECTION("off: no indicator") {
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("REC") == std::string::npos);
    CHECK(out.find("LOG FAIL") == std::string::npos);
  }
  SECTION("recording: REC dot") {
    state.log_status = mgs::LogStatus::Recording;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("REC") != std::string::npos);
  }
  SECTION("failed: loud warning") {
    state.log_status = mgs::LogStatus::Failed;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("LOG FAIL") != std::string::npos);
  }
}

TEST_CASE("dashboard shows mission upload progress", "[ui]") {
  mgs::VehicleState state;

  SECTION("none yet") {
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("Mission") != std::string::npos);
    CHECK(out.find("no mission yet") != std::string::npos);
  }
  SECTION("uploading: items pulled so far") {
    state.mission_status = mgs::MissionStatus::Uploading;
    state.mission_count = 5;
    state.mission_sent = 2;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("2/5") != std::string::npos);
  }
  SECTION("uploaded") {
    state.mission_status = mgs::MissionStatus::Uploaded;
    state.mission_count = 5;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("5 items") != std::string::npos);
  }
  SECTION("rejected: the vehicle's reason in words") {
    state.mission_status = mgs::MissionStatus::Rejected;
    state.mission_error = MAV_MISSION_NO_SPACE;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("too many items") != std::string::npos);
  }
  SECTION("rejected mid-upload: names the refused item") {
    state.mission_status = mgs::MissionStatus::Rejected;
    state.mission_count = 5;
    state.mission_sent = 5;
    state.mission_error = MAV_MISSION_INVALID_PARAM5_X;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("item 5: invalid item") != std::string::npos);
  }
}

TEST_CASE("an 80-column screen fits the hints and a long status", "[ui]") {
  mgs::VehicleState state;
  state.last_command_status = mgs::CommandStatus::Accepted;
  state.last_command_id = MAV_CMD_MISSION_START;
  const std::string out = render_to_text(mgs::render_dashboard(state, true));
  CHECK(out.find("[u]pload") != std::string::npos);
  CHECK(out.find("[q]uit") != std::string::npos);  // the last hint, uncut
  CHECK(out.find("MISSION START accepted") != std::string::npos);
}

namespace {

// A downloaded step: a waypoint at a place unless told otherwise.
mgs::VehicleMissionItem step(std::uint16_t command = MAV_CMD_NAV_WAYPOINT,
                             float alt = 20.0f) {
  mgs::VehicleMissionItem item;
  item.command = command;
  item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  item.alt_m = alt;
  if (command != MAV_CMD_NAV_RETURN_TO_LAUNCH) {
    item.lat_deg = 47.398;
    item.lon_deg = 8.546;
  }
  return item;
}

}  // namespace

TEST_CASE("dashboard lists the vehicle's mission", "[ui]") {
  mgs::VehicleState state;

  SECTION("fetching: items received so far") {
    state.download_status = mgs::DownloadStatus::Downloading;
    state.download_count = 5;
    state.download_received = 3;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("Fetch") != std::string::npos);
    CHECK(out.find("3/5") != std::string::npos);
  }
  SECTION("fetched: each step, numbered from 1, the current one marked") {
    state.download_status = mgs::DownloadStatus::Downloaded;
    state.vehicle_mission = {step(MAV_CMD_NAV_TAKEOFF, 15.0f), step(),
                             step(MAV_CMD_NAV_RETURN_TO_LAUNCH)};
    state.mission_current = 1;  // 0-based: the waypoint
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("1 TAKEOFF") != std::string::npos);
    CHECK(out.find("15 m") != std::string::npos);
    CHECK(out.find("▶  2 WAYPOINT") != std::string::npos);
    CHECK(out.find("3 RTL") != std::string::npos);
  }
  SECTION("a command we do not name still gets a row") {
    state.download_status = mgs::DownloadStatus::Downloaded;
    state.vehicle_mission = {step(MAV_CMD_DO_SET_SERVO)};
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("CMD 183") != std::string::npos);
  }
  SECTION("an empty mission says so") {
    state.download_status = mgs::DownloadStatus::Downloaded;
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("vehicle has no mission") != std::string::npos);
  }
  SECTION("a long mission scrolls to keep the current step in view") {
    state.download_status = mgs::DownloadStatus::Downloaded;
    state.vehicle_mission.assign(20, step());
    state.mission_current = 17;  // step 18
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("▶ 18 WAYPOINT") != std::string::npos);
    CHECK(out.find("  1 WAYPOINT") == std::string::npos);  // scrolled off
  }
  SECTION("upload status and the fetched list share the panel") {
    state.mission_status = mgs::MissionStatus::Uploaded;
    state.mission_count = 1;
    state.download_status = mgs::DownloadStatus::Downloaded;
    state.vehicle_mission = {step()};
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("1 items") != std::string::npos);
    CHECK(out.find("1 WAYPOINT") != std::string::npos);
  }
}

TEST_CASE("the map draws home, the mission, and the vehicle's heading",
          "[ui][map]") {
  mgs::VehicleState state;

  SECTION("nothing to place yet") {
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("Map") != std::string::npos);
  }

  state.has_home = true;
  state.home_lat_deg = 47.3979714;
  state.home_lon_deg = 8.5461636;
  state.has_position = true;
  state.latitude_deg = 47.398102;
  state.longitude_deg = 8.5459;
  state.download_status = mgs::DownloadStatus::Downloaded;
  state.vehicle_mission = {step(), step(MAV_CMD_NAV_RETURN_TO_LAUNCH)};

  SECTION("home, north, a scale bar, and the vehicle as an arrow") {
    state.yaw_deg = 90.0;  // east
    const std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("H") != std::string::npos);
    CHECK(out.find("N│") != std::string::npos);  // top-right of the map
    CHECK(out.find("→") != std::string::npos);
  }
  SECTION("the arrow follows the heading, wrapping past north") {
    state.yaw_deg = -90.0;  // west, as MAVLink may report it
    std::string out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("←") != std::string::npos);
    state.yaw_deg = 359.0;
    out = render_to_text(mgs::render_dashboard(state, true));
    CHECK(out.find("↑") != std::string::npos);
  }
  SECTION("the trail is accepted and drawn without disturbing the rest") {
    const std::vector<mgs::GeoPoint> trail{{47.39797, 8.54616},
                                           {47.39805, 8.54590}};
    const std::string out =
        render_to_text(mgs::render_dashboard(state, true, "", "", trail));
    CHECK(out.find("H") != std::string::npos);
    CHECK(out.find("[q]uit") != std::string::npos);
  }
}
