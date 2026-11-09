#include "map/trail.hpp"

#include <cmath>

namespace mgs {

void Trail::add(GeoPoint at, std::chrono::steady_clock::time_point now) {
  while (!samples_.empty() && now - samples_.front().when > kMaxAge) {
    samples_.pop_front();
  }
  if (!samples_.empty()) {
    const LocalPoint step = to_local(at, samples_.back().at);
    if (std::hypot(step.east_m, step.north_m) < kMinStepM) return;
  }
  samples_.push_back({at, now});
}

std::vector<GeoPoint> Trail::points() const {
  std::vector<GeoPoint> out;
  out.reserve(samples_.size());
  for (const auto& s : samples_) out.push_back(s.at);
  return out;
}

}  // namespace mgs
