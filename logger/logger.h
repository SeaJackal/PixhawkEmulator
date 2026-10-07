#pragma once

#include "simulation_time.h"

#include <sstream>
#include <string>
#include <string_view>
#include <vector>

class Logger {
 public:
  using TimePoint = simulation::TimePoint;

  enum class Level { kDebug, kInfo, kWarning, kError };

  struct Record {
    Level level;
    TimePoint timestamp;
    std::string message;
  };

  explicit Logger(TimePoint simulation_start);

  void Log(Level level, TimePoint timestamp, std::string_view message);
  [[nodiscard]] std::string GetText() const;
  std::vector<Record> TakeRecords();
  static std::string FormatRecord(const Record& record, TimePoint simulation_start);
  void Clear();

 private:
  TimePoint simulation_start_;
  std::ostringstream buffer_;
  std::vector<Record> records_;
};
