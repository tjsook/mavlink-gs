#include <catch2/catch_test_macros.hpp>

#include "ui/step_entry.hpp"

TEST_CASE("choose_step turns a 1-based step into a 0-based seq", "[goto]") {
  const auto first = mgs::choose_step("1", 5);
  REQUIRE(first.seq);
  CHECK(*first.seq == 0);

  const auto last = mgs::choose_step("5", 5);
  REQUIRE(last.seq);
  CHECK(*last.seq == 4);

  CHECK(*mgs::choose_step("007", 10).seq == 6);  // leading zeros are fine
}

TEST_CASE("choose_step refuses steps the mission does not have", "[goto]") {
  const auto zero = mgs::choose_step("0", 5);
  CHECK_FALSE(zero.seq);
  CHECK(zero.error == "no step 0 (the mission has 1–5)");

  const auto past = mgs::choose_step("6", 5);
  CHECK_FALSE(past.seq);
  CHECK(past.error == "no step 6 (the mission has 1–5)");
}

TEST_CASE("choose_step needs a mission and a number", "[goto]") {
  CHECK(mgs::choose_step("1", 0).error == "no mission on the vehicle");
  CHECK(mgs::choose_step("", 5).error == "type a step number");
  CHECK_FALSE(mgs::choose_step("2a", 5).seq);
  CHECK_FALSE(mgs::choose_step("123456", 200000).seq);  // too many digits
}
