#include "map/geo.hpp"

#include <algorithm>
#include <cmath>

namespace mgs {
namespace {

// Metres per degree of latitude (and of longitude at the equator), from the
// mean Earth radius: 6371 km * pi / 180.
constexpr double kMetresPerDegree = 6'371'000.0 * M_PI / 180.0;

}  // namespace

LocalPoint to_local(GeoPoint p, GeoPoint origin) {
  const double cos_lat = std::cos(origin.lat_deg * M_PI / 180.0);
  return {(p.lon_deg - origin.lon_deg) * kMetresPerDegree * cos_lat,
          (p.lat_deg - origin.lat_deg) * kMetresPerDegree};
}

MapFit::MapFit(const std::vector<LocalPoint>& points, int width, int height,
               int margin) {
  auto [min_e, max_e] = std::minmax_element(
      points.begin(), points.end(),
      [](const LocalPoint& a, const LocalPoint& b) { return a.east_m < b.east_m; });
  auto [min_n, max_n] = std::minmax_element(
      points.begin(), points.end(),
      [](const LocalPoint& a, const LocalPoint& b) { return a.north_m < b.north_m; });

  centre_ = {(min_e->east_m + max_e->east_m) / 2.0,
             (min_n->north_m + max_n->north_m) / 2.0};
  const double span_e = std::max(max_e->east_m - min_e->east_m, kMinSpanM);
  const double span_n = std::max(max_n->north_m - min_n->north_m, kMinSpanM);

  // The tighter axis sets the scale; the other gets spare room either side.
  const int usable_w = std::max(width - 1 - 2 * margin, 1);
  const int usable_h = std::max(height - 1 - 2 * margin, 1);
  metres_per_dot_ = std::max(span_e / usable_w, span_n / usable_h);
  centre_x_ = (width - 1) / 2.0;
  centre_y_ = (height - 1) / 2.0;
}

int MapFit::x(LocalPoint p) const {
  return static_cast<int>(
      std::lround(centre_x_ + (p.east_m - centre_.east_m) / metres_per_dot_));
}

int MapFit::y(LocalPoint p) const {
  return static_cast<int>(
      std::lround(centre_y_ - (p.north_m - centre_.north_m) / metres_per_dot_));
}

double nice_scale_length(double max_m) {
  if (max_m <= 0.0) return 0.0;
  const double power = std::pow(10.0, std::floor(std::log10(max_m)));
  for (const double step : {5.0, 2.0, 1.0}) {
    if (step * power <= max_m) return step * power;
  }
  return power;  // unreachable: 1 * power <= max_m by construction
}

}  // namespace mgs
