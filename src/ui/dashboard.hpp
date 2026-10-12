#pragma once

#include <string>

#include <ftxui/dom/elements.hpp>

#include "state/vehicle_state.hpp"

namespace mgs {

// Build the full dashboard view from a telemetry snapshot.
//
// Pure: it reads the snapshot and returns an FTXUI element tree; it touches no
// shared state, no socket, and no clock. `connected` is derived by the caller
// from heartbeat freshness (see main) and passed in, so this function stays
// testable with plain values.
//
// `confirm_prompt`, when non-empty, replaces the command hint bar with a
// highlighted confirmation question (e.g. "TAKEOFF? confirm (y/n)"). `note` is
// a transient one-line message (e.g. "no vehicle yet"). Both default to empty
// so telemetry-only callers and tests need not supply them.
ftxui::Element render_dashboard(const VehicleState& state, bool connected,
                                const std::string& confirm_prompt = "",
                                const std::string& note = "");

}  // namespace mgs
