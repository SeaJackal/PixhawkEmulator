#pragma once

#include <string>

#include "logger.h"
#include "simulation_time.h"
#include "udp_transport.h"

std::string FormatSyslog(const Logger::Record& record,
                         simulation::TimePoint simulation_start);

class SyslogSender {
 public:
  SyslogSender(const UdpEndpoint& collector,
               simulation::TimePoint simulation_start);
  void Send(const Logger::Record& record);

 private:
  simulation::TimePoint simulation_start_;
  UdpTransport transport_;
};
