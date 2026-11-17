#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <istream>
#include <ostream>

namespace mgs {

// What a conversion did, for the caller to report.
struct CsvExportResult {
  std::size_t rows = 0;      // data rows written (not counting the header)
  std::size_t skipped = 0;   // .tlog frames dropped for a bad checksum
  bool truncated = false;    // the .tlog ended mid-record (e.g. a crash)
};

// One CSV row per this much log time, at most.
inline constexpr auto kCsvPeriod = std::chrono::milliseconds(100);

// Turn a .tlog into a telemetry CSV, one row per kCsvPeriod.
//
// The .tlog stays the source of truth; the CSV is a view of it. Each record is
// folded through the same update() the dashboard uses, so a CSV row shows what
// the dashboard showed at that moment. Our own sent messages (GCS heartbeats,
// commands) are in the .tlog too, and are left out: they describe us, not the
// vehicle. Rows start once the vehicle has been heard.
//
// Columns: time_s (since the first record), utc, armed, lat_deg, lon_deg,
// alt_msl_m, rel_alt_m, roll_deg, pitch_deg, yaw_deg, battery_v, battery_a,
// battery_pct, mission_step (1-based). A value not known yet is left empty.
CsvExportResult tlog_to_csv(std::istream& tlog, std::ostream& csv);

// The CSV path beside a .tlog: same name, .csv extension.
std::filesystem::path csv_path_for(const std::filesystem::path& tlog);

}  // namespace mgs
