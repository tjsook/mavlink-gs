#include <chrono>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "map/geo.hpp"
#include "map/trail.hpp"

using Catch::Matchers::WithinAbs;

TEST_CASE("to_local turns degrees into metres east and north", "[map]") {
  const mgs::GeoPoint origin{47.0, 8.0};

  const auto north = mgs::to_local({47.001, 8.0}, origin);
  CHECK_THAT(north.north_m, WithinAbs(111.19, 0.01));  // 0.001 deg of latitude
  CHECK_THAT(north.east_m, WithinAbs(0.0, 1e-9));

  // A degree of longitude is shorter away from the equator: at 60 deg, half.
  const auto east = mgs::to_local({60.0, 8.001}, {60.0, 8.0});
  CHECK_THAT(east.east_m, WithinAbs(111.19 / 2, 0.01));

  const auto sw = mgs::to_local({46.999, 7.999}, origin);
  CHECK(sw.east_m < 0);
  CHECK(sw.north_m < 0);
}

TEST_CASE("MapFit keeps one scale for both axes", "[map]") {
  // A 100 m square on a wide canvas: the height limits the scale, and the
  // square still draws as a square (equal width and height in dots).
  const std::vector<mgs::LocalPoint> square{{0, 0}, {100, 0}, {100, 100}, {0, 100}};
  const mgs::MapFit fit(square, 200, 81, 0);

  const int w = fit.x({100, 0}) - fit.x({0, 0});
  const int h = fit.y({0, 0}) - fit.y({0, 100});
  CHECK(w == h);
  CHECK(h == 80);  // fills the full height
  CHECK(fit.y({0, 100}) < fit.y({0, 0}));  // north is up
}

TEST_CASE("MapFit centres the points and keeps the margin clear", "[map]") {
  const std::vector<mgs::LocalPoint> line{{-50, 10}, {150, 10}};
  const mgs::MapFit fit(line, 101, 41, 5);
  CHECK(fit.x({-50, 10}) == 5);
  CHECK(fit.x({150, 10}) == 95);
  CHECK(fit.y({0, 10}) == 20);  // the middle row
}

TEST_CASE("MapFit does not zoom in past kMinSpanM", "[map]") {
  // One point: no span at all. The scale falls back to kMinSpanM across.
  const mgs::MapFit fit({{3, 4}}, 41, 41, 0);
  CHECK_THAT(fit.metres_per_dot(), WithinAbs(mgs::MapFit::kMinSpanM / 40, 1e-9));
  CHECK(fit.x({3, 4}) == 20);
  CHECK(fit.y({3, 4}) == 20);
}

TEST_CASE("nice_scale_length picks 1, 2 or 5 times a power of ten", "[map]") {
  CHECK(mgs::nice_scale_length(37) == 20);
  CHECK(mgs::nice_scale_length(120) == 100);
  CHECK(mgs::nice_scale_length(50) == 50);
  CHECK_THAT(mgs::nice_scale_length(4.9), WithinAbs(2.0, 1e-9));
  CHECK(mgs::nice_scale_length(0) == 0);
}

TEST_CASE("Trail keeps moves, skips jitter, and forgets old samples", "[map]") {
  using namespace std::chrono_literals;
  const auto t0 = std::chrono::steady_clock::time_point{} + 100s;
  mgs::Trail trail;

  trail.add({47.0, 8.0}, t0);
  trail.add({47.0000001, 8.0}, t0 + 100ms);  // ~1 cm: parked, not moving
  CHECK(trail.points().size() == 1);

  trail.add({47.0001, 8.0}, t0 + 1s);  // ~11 m north
  CHECK(trail.points().size() == 2);

  // Past kMaxAge, the oldest samples drop off, even if the vehicle is parked.
  trail.add({47.0001, 8.0}, t0 + mgs::Trail::kMaxAge + 500ms);
  REQUIRE(trail.points().size() == 1);
  CHECK(trail.points().front().lat_deg == 47.0001);
}
