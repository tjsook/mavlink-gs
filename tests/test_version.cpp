#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "core/version.hpp"

TEST_CASE("version() returns a non-empty string", "[version]") {
  const std::string_view v{mgs::version()};
  REQUIRE_FALSE(v.empty());
}
