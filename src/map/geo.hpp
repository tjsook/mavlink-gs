#pragma once

#include <vector>

namespace mgs {

// A place on Earth, in degrees.
struct GeoPoint {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
};

// A place on a flat map, in metres east and north of some origin.
struct LocalPoint {
  double east_m = 0.0;
  double north_m = 0.0;
};

// Project `p` onto a flat map centred on `origin`.
//
// Treats the Earth as flat near the origin ("equirectangular"): a degree of
// latitude is a fixed distance, and a degree of longitude shrinks with
// cos(latitude). Over a mission a few hundred metres across the error is
// centimetres — far below one map dot — so the full spherical maths buys
// nothing here.
LocalPoint to_local(GeoPoint p, GeoPoint origin);

// Fits a set of points onto a canvas of width x height dots, with one scale for
// both axes, so a square mission draws as a square.
//
// A Braille cell is 2 dots wide and 4 tall, and a terminal cell is about twice
// as tall as wide, so each dot is close to square: one scale fits both axes.
// North is up: dot y grows downward, so y falls as north_m rises.
class MapFit {
 public:
  // Never zoom in closer than this many metres across, so a lone parked
  // vehicle is not blown up to fill the screen with sensor noise.
  static constexpr double kMinSpanM = 20.0;

  // `points` must not be empty. `margin` dots stay clear on every side, for
  // labels drawn beside the outermost points.
  MapFit(const std::vector<LocalPoint>& points, int width, int height,
         int margin);

  int x(LocalPoint p) const;  // dot column
  int y(LocalPoint p) const;  // dot row
  double metres_per_dot() const { return metres_per_dot_; }

 private:
  LocalPoint centre_;
  double metres_per_dot_ = 1.0;
  double centre_x_ = 0.0;  // the canvas centre, in dots
  double centre_y_ = 0.0;
};

// The longest "round" length (1, 2 or 5 times a power of ten) that is no more
// than `max_m`, for a scale bar: 37 -> 20, 120 -> 100, 4.9 -> 2.
double nice_scale_length(double max_m);

}  // namespace mgs
