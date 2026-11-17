#include "log/csv_export.hpp"

#include <ctime>
#include <optional>
#include <string>

#include <spdlog/fmt/fmt.h>

#include "log/tlog_reader.hpp"
#include "mavlink/commands.hpp"  // kGcsSystemId
#include "state/vehicle_state.hpp"

namespace mgs {
namespace {

constexpr const char* kHeader =
    "time_s,utc,armed,lat_deg,lon_deg,alt_msl_m,rel_alt_m,roll_deg,pitch_deg,"
    "yaw_deg,battery_v,battery_a,battery_pct,mission_step\n";

// ISO 8601 UTC with milliseconds, e.g. 2026-11-16T19:55:03.123Z.
std::string utc(std::chrono::system_clock::time_point t) {
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      t.time_since_epoch())
                      .count();
  const std::time_t secs = static_cast<std::time_t>(ms / 1000);
  std::tm tm{};
  gmtime_r(&secs, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  return fmt::format("{}.{:03}Z", buf, ms % 1000);
}

void write_row(std::ostream& csv, const VehicleState& s, double time_s,
               std::chrono::system_clock::time_point when) {
  // Unknown values stay empty, not 0: a 0 latitude is a real place.
  const std::string pos =
      s.has_position
          ? fmt::format("{:.7f},{:.7f},{:.2f},{:.2f}", s.latitude_deg,
                        s.longitude_deg, s.altitude_m, s.relative_alt_m)
          : ",,,";
  const std::string pct =
      s.battery_remaining >= 0 ? std::to_string(s.battery_remaining) : "";
  const std::string step =
      s.mission_current >= 0 ? std::to_string(s.mission_current + 1) : "";
  csv << fmt::format("{:.3f},{},{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{}\n",
                     time_s, utc(when), s.armed ? 1 : 0, pos, s.roll_deg,
                     s.pitch_deg, s.yaw_deg, s.battery_volts, s.battery_amps,
                     pct, step);
}

}  // namespace

CsvExportResult tlog_to_csv(std::istream& tlog, std::ostream& csv) {
  CsvExportResult result;
  TlogReader reader(tlog);
  VehicleState state;
  std::optional<std::chrono::system_clock::time_point> first;
  std::chrono::system_clock::time_point next_row{};

  csv << kHeader;
  while (const auto rec = reader.next()) {
    if (!first) first = rec->time;
    if (rec->msg.sysid == kGcsSystemId) continue;  // our own, not the vehicle

    // update() wants a steady_clock time for heartbeat freshness; that field
    // is not in the CSV, so the replay passes a fixed stand-in.
    update(state, rec->msg, std::chrono::steady_clock::time_point{});

    if (!state.identified() || rec->time < next_row) continue;
    const std::chrono::duration<double> since = rec->time - *first;
    write_row(csv, state, since.count(), rec->time);
    ++result.rows;
    // Spaced by log time, not by record count: after a gap (a laptop that
    // slept) the next row comes with the next record, not a burst of copies.
    next_row = rec->time + kCsvPeriod;
  }
  result.skipped = reader.skipped();
  result.truncated = reader.truncated();
  return result;
}

std::filesystem::path csv_path_for(const std::filesystem::path& tlog) {
  std::filesystem::path out = tlog;
  return out.replace_extension(".csv");
}

}  // namespace mgs
