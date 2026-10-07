#include "syslog.h"

#include <stdexcept>
#include <string_view>

namespace {

int Severity(Logger::Level level) {
  switch (level) {
    case Logger::Level::kDebug: return 7;
    case Logger::Level::kInfo: return 6;
    case Logger::Level::kWarning: return 4;
    case Logger::Level::kError: return 3;
  }
  throw std::invalid_argument("Unknown log level");
}

}

std::string FormatSyslog(const Logger::Record& record,
                         simulation::TimePoint simulation_start) {
  const auto header = "<" + std::to_string(16 * 8 + Severity(record.level)) +
      ">1 - - pixhawk_simulator - - - ";
  auto body = Logger::FormatRecord(record, simulation_start);
  constexpr std::size_t kMaximumBytes = 480;
  constexpr std::string_view kTruncated = " [truncated]";
  const auto available = kMaximumBytes - header.size();
  if (body.size() > available) {
    body.resize(available - kTruncated.size());
    body.append(kTruncated);
  }
  return header + body;
}

SyslogSender::SyslogSender(const UdpEndpoint& collector,
                           simulation::TimePoint simulation_start)
    : simulation_start_(simulation_start),
      transport_({"0.0.0.0", 0}, collector) {}

void SyslogSender::Send(const Logger::Record& record) {
  const auto text = FormatSyslog(record, simulation_start_);
  const auto* begin = reinterpret_cast<const std::byte*>(text.data());
  transport_.Send(std::vector<std::byte>(begin, begin + text.size()));
}
