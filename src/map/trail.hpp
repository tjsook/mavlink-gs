#pragma once

#include <chrono>
#include <deque>
#include <vector>

#include "map/geo.hpp"

namespace mgs {

// Where the vehicle has been recently, for the map to draw behind it.
//
// Fed from the UI thread's own snapshots, so the network thread never knows
// it exists. A sample is kept only if the vehicle moved since the last one —
// a parked vehicle does not fill the trail with copies of one spot — and
// samples older than kMaxAge fall off the tail.
class Trail {
 public:
  static constexpr auto kMaxAge = std::chrono::seconds(60);
  static constexpr double kMinStepM = 0.5;

  void add(GeoPoint at, std::chrono::steady_clock::time_point now);
  std::vector<GeoPoint> points() const;  // oldest first

 private:
  struct Sample {
    GeoPoint at;
    std::chrono::steady_clock::time_point when;
  };
  std::deque<Sample> samples_;
};

}  // namespace mgs
