#pragma once

#include <vector>

#include <ftxui/dom/elements.hpp>

#include "map/geo.hpp"
#include "state/vehicle_state.hpp"

namespace mgs {

// A north-up Braille map of the flight: the mission path with numbered steps,
// home, the recent trail, and the vehicle as an arrow along its heading.
//
// Pure, like render_dashboard(): the snapshot and trail in, an element out.
// The view zooms itself to hold everything it draws, and the canvas takes
// whatever space the layout gives it, so a taller terminal gets a bigger map.
ftxui::Element render_map(const VehicleState& state,
                          const std::vector<GeoPoint>& trail);

}  // namespace mgs
