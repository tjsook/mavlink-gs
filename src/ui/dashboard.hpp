#pragma once

#include <ftxui/dom/elements.hpp>

#include "state/vehicle_state.hpp"

namespace mgs {

// Build the full dashboard view from a telemetry snapshot.
//
// Pure: it reads the snapshot and returns an FTXUI element tree; it touches no
// shared state, no socket, and no clock. `connected` is derived by the caller
// from heartbeat freshness (see main) and passed in, so this function stays
// testable with plain values.
ftxui::Element render_dashboard(const VehicleState& state, bool connected);

}  // namespace mgs
