#include <string>

#include <catch2/catch_test_macros.hpp>
#include <ftxui/dom/node.hpp>
#include <ftxui/screen/screen.hpp>

#include "state/vehicle_state.hpp"
#include "ui/dashboard.hpp"

namespace {

// Render an element to a fixed-size character grid and flatten to text so we
// can assert on the content the user would see.
std::string render_to_text(const ftxui::Element& element) {
  auto screen = ftxui::Screen::Create(ftxui::Dimension::Fixed(80),
                                      ftxui::Dimension::Fixed(24));
  // Render mutates a copy of the shared element tree; take one so `element`
  // stays const-correct at the call site.
  ftxui::Element copy = element;
  ftxui::Render(screen, copy);
  return screen.ToString();
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
