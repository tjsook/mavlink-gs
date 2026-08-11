#include <string>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include "core/version.hpp"

int main(int argc, char** argv) {
  CLI::App app{"mavlink-gs — terminal-based MAVLink ground station"};
  app.set_version_flag("--version", mgs::version());

  std::string connection;
  app.add_option("--connect", connection,
                 "Connection URL, e.g. udp://127.0.0.1:14550");

  CLI11_PARSE(app, argc, argv);

  spdlog::info("mavlink-gs {} starting", mgs::version());
  if (connection.empty()) {
    spdlog::warn("no --connect target given; nothing to do yet");
  } else {
    spdlog::info("connection target: {}", connection);
  }

  // TODO(v0.1): open the connection and start decoding telemetry.
  return 0;
}
