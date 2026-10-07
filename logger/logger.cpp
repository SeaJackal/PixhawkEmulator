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
  records_.push_back({level, timestamp, std::string(message)});
  buffer_ << FormatRecord(records_.back(), simulation_start_);
}

std::string Logger::FormatRecord(const Record& record, TimePoint simulation_start) {
  const auto seconds =
      std::chrono::duration<double>(record.timestamp - simulation_start).count();
  std::ostringstream text;
  text << std::fixed << std::setprecision(9) << '[' << seconds << "] ["
       << LevelName(record.level) << "] " << record.message << '\n';
  return text.str();
}

std::string Logger::GetText() const { return buffer_.str(); }

std::vector<Logger::Record> Logger::TakeRecords() {
  std::vector<Record> records;
  records.swap(records_);
  return records;
}

void Logger::Clear() {
  buffer_.str("");
  buffer_.clear();
  records_.clear();
}
