#include "ui/step_entry.hpp"

#include <spdlog/fmt/fmt.h>

namespace mgs {

StepChoice choose_step(std::string_view typed, std::size_t mission_size) {
  if (mission_size == 0) return {std::nullopt, "no mission on the vehicle"};
  if (typed.empty()) return {std::nullopt, "type a step number"};

  // The prompt only lets digits in, at most kMaxStepDigits of them, so this
  // cannot overflow; check anyway rather than trust the caller.
  std::size_t step = 0;
  for (const char c : typed) {
    if (c < '0' || c > '9' || typed.size() > kMaxStepDigits) {
      return {std::nullopt, "type a step number"};
    }
    step = step * 10 + static_cast<std::size_t>(c - '0');
  }
  if (step < 1 || step > mission_size) {
    return {std::nullopt,
            fmt::format("no step {} (the mission has 1–{})", step,
                        mission_size)};
  }
  return {static_cast<std::uint16_t>(step - 1), ""};
}

}  // namespace mgs
