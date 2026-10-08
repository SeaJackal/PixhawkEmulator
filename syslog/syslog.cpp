#include "syslog.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "logger.h"
#include "simulation_time.h"
#include "udp_transport.h"

namespace {

int Severity(Logger::Level level);

// -----------------------------------------------------------------------------

int Severity(Logger::Level level) {
  constexpr int kDebugSeverity = 7;
  constexpr int kInfoSeverity = 6;
  switch (level) {
    case Logger::Level::kDebug:
      return kDebugSeverity;
    case Logger::Level::kInfo:
      return kInfoSeverity;
    case Logger::Level::kWarning:
      return 4;
    case Logger::Level::kError:
      return 3;
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
  std::vector<std::byte> bytes;
  bytes.reserve(text.size());
  std::transform(
      text.begin(), text.end(), std::back_inserter(bytes),
      [](unsigned char value) { return static_cast<std::byte>(value); });
  transport_.Send(bytes);
}
