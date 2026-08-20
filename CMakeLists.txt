cmake_minimum_required(VERSION 3.25)

project(mavlink-gs
  VERSION 0.1.0
  DESCRIPTION "Terminal-based ground station for MAVLink vehicles"
  LANGUAGES CXX)

# --- Language standard -------------------------------------------------------
# C++20 baseline, targeting C++23 features where available. STANDARD_REQUIRED
# makes it a hard error (not a silent downgrade) if the compiler can't do C++20.
# EXTENSIONS OFF asks for -std=c++20 rather than -std=gnu++20 (portable, no
# compiler-specific extensions).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json so clangd / clang-tidy understand the build.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Dependencies (installed by vcpkg from vcpkg.json) -----------------------
# CONFIG mode uses each library's own shipped <Lib>Config.cmake; REQUIRED fails
# loudly at configure time if a package is missing.
find_package(asio     CONFIG REQUIRED)
find_package(ftxui    CONFIG REQUIRED)
find_package(CLI11    CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(spdlog   CONFIG REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)

# --- MAVLink headers (vendored submodule, header-only) -----------------------
# An INTERFACE library carries no compiled code; it just propagates the include
# path to anything that links it. This is the modern replacement for a global
# include_directories().
add_library(mavlink INTERFACE)
target_include_directories(mavlink INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/external/mavlink)

# --- Core library: all real logic lives here ---------------------------------
# The executable and the tests both link this one library, so code is compiled
# once and is testable without duplicating sources.
add_library(mavlink-gs-core
  src/core/version.cpp)

# PUBLIC include dir: both this library's .cpp files and its consumers can
# #include "core/...". PUBLIC because these headers are part of the lib's API.
target_include_directories(mavlink-gs-core PUBLIC src)

# Stamp the CMake project version into the binary. PRIVATE: only version.cpp
# uses it, it doesn't leak through any header.
target_compile_definitions(mavlink-gs-core PRIVATE
  MGS_VERSION="${PROJECT_VERSION}")

# asio + mavlink are PUBLIC: they will appear in core's public headers later, so
# consumers need their include paths. spdlog is PRIVATE: an internal logging
# detail that never shows up in core's headers.
target_link_libraries(mavlink-gs-core
  PUBLIC  mavlink asio::asio
  PRIVATE spdlog::spdlog)

# --- Executable: a thin main() over the core library -------------------------
# Nothing links an executable, so every dependency here is PRIVATE.
add_executable(mavlink-gs src/main.cpp)
target_link_libraries(mavlink-gs PRIVATE
  mavlink-gs-core
  CLI11::CLI11
  spdlog::spdlog
  yaml-cpp::yaml-cpp
  ftxui::screen ftxui::dom ftxui::component)

# --- Tests -------------------------------------------------------------------
enable_testing()
add_executable(mavlink-gs-tests tests/test_version.cpp)
# Catch2WithMain provides Catch2's own main(), so tests need no main.cpp.
target_link_libraries(mavlink-gs-tests PRIVATE
  mavlink-gs-core
  Catch2::Catch2WithMain)

# catch_discover_tests registers each TEST_CASE with CTest individually.
# The Catch module becomes available on CMAKE_MODULE_PATH after find_package(Catch2).
include(Catch)
catch_discover_tests(mavlink-gs-tests)
