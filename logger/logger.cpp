#include "logger.h"

#include <iomanip>

namespace {

const char* LevelName(Logger::Level level) {
  switch (level) {
    case Logger::Level::kDebug:
      return "DEBUG";
    case Logger::Level::kInfo:
      return "INFO";
    case Logger::Level::kWarning:
      return "WARNING";
    case Logger::Level::kError:
      return "ERROR";
  }
  return "UNKNOWN";
}

}

Logger::Logger(TimePoint simulation_start) : simulation_start_(simulation_start) {}

void Logger::Log(Level level, TimePoint timestamp, std::string_view message) {
  const auto seconds =
      std::chrono::duration<double>(timestamp - simulation_start_).count();
  buffer_ << std::fixed << std::setprecision(9) << '[' << seconds << "] ["
          << LevelName(level) << "] " << message << '\n';
}

std::string Logger::GetText() const { return buffer_.str(); }

void Logger::Clear() {
  buffer_.str("");
  buffer_.clear();
}
