#include "controller.h"
#include "simulation_config.h"
#include "simulation_runner.h"
#include "syslog.h"
#include "udp_transport.h"

#include <charconv>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void Stop(int) {
  stop_requested = 1;
}

int Number(std::string_view text, int maximum) {
  int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      value < 0 || value > maximum) {
    throw std::invalid_argument("Invalid numeric option: " + std::string(text));
  }
  return value;
}

}

int main(int argc, char** argv) try {
  const char* config_path = nullptr;
  for (int index = 1; index < argc; ++index) {
    const std::string_view option = argv[index];
    if (option == "--help") {
      std::cout << "pixhawk_simulator [--config FILE]\n"
                   "                  [--bind-address IPv4] [--bind-port PORT]\n"
                   "                  [--manual-bind-address IPv4] [--manual-bind-port PORT]\n"
                   "                  [--peer-address IPv4] [--peer-port PORT]\n"
                   "                  [--inbound-latency-ms MS] [--outbound-latency-ms MS]\n"
                   "Command-line options override config file values.\n";
      return 0;
    }
    if (++index == argc) throw std::invalid_argument("Missing option value");
    if (option == "--config") {
      if (config_path) throw std::invalid_argument("Duplicate --config option");
      config_path = argv[index];
    }
  }
  auto config = config_path ? LoadSimulationConfig(config_path) : SimulationConfig{};
  auto& local = config.transport.local;
  auto& manual_local = config.transport.manual_local;
  auto& peer = config.transport.peer;
  for (int index = 1; index < argc; index += 2) {
    const std::string_view option = argv[index];
    if (option == "--config") continue;
    const std::string_view value = argv[index + 1];
    if (option == "--bind-address") local.address = value;
    else if (option == "--manual-bind-address") manual_local.address = value;
    else if (option == "--manual-bind-port") manual_local.port = Number(value, 65535);
    else if (option == "--peer-address") peer.address = value;
    else if (option == "--bind-port") local.port = Number(value, 65535);
    else if (option == "--peer-port") peer.port = Number(value, 65535);
    else if (option == "--inbound-latency-ms") {
      config.network.inbound.latency = std::chrono::milliseconds{Number(value, 2147483647)};
    } else if (option == "--outbound-latency-ms") {
      config.network.outbound.latency = std::chrono::milliseconds{Number(value, 2147483647)};
    } else {
      throw std::invalid_argument("Unknown option: " + std::string(option));
    }
  }
  if (peer.port == 0) throw std::invalid_argument("Peer port must be nonzero");
  DynamicModel model{config.dynamic_model.linear_speed_meters_per_second,
                     config.dynamic_model.angular_speed_radians_per_second};
  BatteryModel battery{config.battery.max_capacity_amp_hours,
                       config.battery.start_capacity_amp_hours};
  std::chrono::steady_clock clock;
  const auto start = clock.now();
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start,
                                       config.state_machine.readiness_timeout};
  Controller controller{model, battery, state_machine, start};
  for (const auto& loss : config.controller.gnss_losses) {
    if (start > simulation::TimePoint::max() - loss.start) {
      throw std::invalid_argument("GNSS loss start exceeds simulation clock range");
    }
    controller.AddGnssLoss(start + loss.start, loss.duration);
  }
  UdpTransport transport{local, peer};
  UdpTransport manual_transport{manual_local, peer};
  std::unique_ptr<SyslogSender> syslog;
  std::function<void(const Logger::Record&)> record_sink;
  if (config.syslog) {
    try {
      syslog = std::make_unique<SyslogSender>(*config.syslog, start);
      record_sink = [&syslog](const Logger::Record& record) {
        syslog->Send(record);
      };
    } catch (const std::exception& error) {
      std::cerr << "syslog disabled: " << error.what() << '\n';
    }
  }
  SimulationRunner runner{controller, transport, manual_transport,
                          CreateNetworkEmulator(config.network.inbound, start),
                          CreateNetworkEmulator(config.network.outbound, start),
                          logger, std::move(record_sink)};
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  const auto bound = transport.LocalEndpoint();
  const auto manual_bound = manual_transport.LocalEndpoint();
  std::cout << "Automatic listening on " << bound.address << ':' << bound.port
            << "; manual listening on " << manual_bound.address << ':' << manual_bound.port
            << "; peer " << peer.address << ':' << peer.port
            << "; timestamps use steady_clock\n" << std::flush;
  runner.Run([] { return stop_requested != 0; });
} catch (const std::exception& error) {
  std::cerr << "pixhawk_simulator: " << error.what() << '\n';
  return 1;
}
