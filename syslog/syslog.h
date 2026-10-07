#pragma once

#include "logger.h"
#include "udp_transport.h"

#include <string>

std::string FormatSyslog(const Logger::Record& record,
                         simulation::TimePoint simulation_start);

class SyslogSender {
 public:
  SyslogSender(const UdpEndpoint& collector, simulation::TimePoint simulation_start);
  void Send(const Logger::Record& record);

 private:
  simulation::TimePoint simulation_start_;
  UdpTransport transport_;
};
