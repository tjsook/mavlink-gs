#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "state/shared_state.hpp"

TEST_CASE("snapshot returns an independent copy", "[state]") {
  mgs::SharedState shared;
  shared.modify([](mgs::VehicleState& s) { s.battery_remaining = 50; });

  mgs::VehicleState first = shared.snapshot();
  shared.modify([](mgs::VehicleState& s) { s.battery_remaining = 90; });

  // The earlier snapshot is a copy, so the later write does not touch it.
  CHECK(first.battery_remaining == 50);
  CHECK(shared.snapshot().battery_remaining == 90);
}

TEST_CASE("concurrent writers and readers stay consistent", "[state]") {
  mgs::SharedState shared;
  constexpr int kWriters = 4;
  constexpr int kPerWriter = 10000;

  std::vector<std::thread> writers;
  for (int w = 0; w < kWriters; ++w) {
    writers.emplace_back([&shared] {
      for (int i = 0; i < kPerWriter; ++i) {
        // Read-modify-write of a shared field; only safe because modify() holds
        // the lock across the whole lambda.
        shared.modify([](mgs::VehicleState& s) { ++s.custom_mode; });
      }
    });
  }

  // Hammer snapshot() from this thread while the writers run. Under the ASan/
  // TSan build this is where an unguarded read would be caught.
  for (int i = 0; i < kPerWriter; ++i) {
    volatile auto sink = shared.snapshot().custom_mode;
    (void)sink;
  }

  for (auto& t : writers) t.join();

  // Every increment was applied exactly once: no lost updates.
  CHECK(shared.snapshot().custom_mode ==
        static_cast<std::uint32_t>(kWriters * kPerWriter));
}
