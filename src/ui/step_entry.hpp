#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace mgs {

// What a typed step number resolves to: the 0-based mission seq to send, or
// (seq empty) a message saying why it cannot be sent.
struct StepChoice {
  std::optional<std::uint16_t> seq;
  std::string error;
};

// Most digits the goto prompt accepts: a mission holds at most 65535 items.
inline constexpr std::size_t kMaxStepDigits = 5;

// Resolve what the user typed after [g] against a mission of `mission_size`
// steps. People count steps from 1, as the YAML and the Mission panel do;
// MAVLink counts from 0. This is the one place that converts.
StepChoice choose_step(std::string_view typed, std::size_t mission_size);

}  // namespace mgs
