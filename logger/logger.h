#pragma once

#include "simulation_time.h"

#include <sstream>
#include <string>
#include <string_view>

class Logger {
 public:
  using TimePoint = simulation::TimePoint;

  enum class Level { kDebug, kInfo, kWarning, kError };

  explicit Logger(TimePoint simulation_start);

  void Log(Level level, TimePoint timestamp, std::string_view message);
  [[nodiscard]] std::string GetText() const;
  void Clear();

 private:
  TimePoint simulation_start_;
  std::ostringstream buffer_;
};
