#include <charconv>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "battery_model.h"
#include "controller.h"
#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "logger.h"
#include "simulation_config.h"
#include "simulation_runner.h"
#include "simulation_time.h"
#include "syslog.h"
#include "udp_transport.h"

namespace {

class ShutdownSignal {
 public:
  static void Stop([[maybe_unused]] int signal);
  static bool Requested();

 private:
  static inline volatile std::sig_atomic_t requested = 0;
};

int Number(std::string_view text, int maximum);

std::optional<SimulationConfig> ReadOptions(
    const std::vector<char*>& arguments);

// -----------------------------------------------------------------------------

void ShutdownSignal::Stop([[maybe_unused]] int signal) { requested = 1; }

bool ShutdownSignal::Requested() { return requested != 0; }

int Number(std::string_view text, int maximum) {
  int value = 0;
  const auto* end = text.end();
  const auto result = std::from_chars(text.begin(), end, value);
  if (result.ec != std::errc{} || result.ptr != end || value < 0 ||
      value > maximum) {
    throw std::invalid_argument("Invalid numeric option: " + std::string(text));
  }
  return value;
}

std::optional<SimulationConfig> ReadOptions(
    const std::vector<char*>& arguments) {
  const char* config_path = nullptr;
  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const std::string_view option = arguments[index];
    if (option == "--help") {
      std::cout
          << "pixhawk_simulator [--config FILE]\n"
             "                  [--bind-address IPv4] [--bind-port PORT]\n"
             "                  [--manual-bind-address IPv4] "
             "[--manual-bind-port PORT]\n"
             "                  [--peer-address IPv4] [--peer-port PORT]\n"
             "                  [--inbound-latency-ms MS] "
             "[--outbound-latency-ms MS]\n"
             "Command-line options override config file values.\n";
      return std::nullopt;
    }
    if (++index == arguments.size()) {
      throw std::invalid_argument("Missing option value");
    }
    if (option == "--config") {
      if (config_path != nullptr) {
        throw std::invalid_argument("Duplicate --config option");
      }
      config_path = arguments[index];
    }
  }
  auto config = (config_path != nullptr) ? LoadSimulationConfig(config_path)
                                         : SimulationConfig{};
  auto& local = config.transport.local;
  auto& manual_local = config.transport.manual_local;
  auto& peer = config.transport.peer;
  for (std::size_t index = 1; index < arguments.size(); index += 2) {
    const std::string_view option = arguments[index];
    if (option == "--config") {
      continue;
    }
    const std::string_view value = arguments[index + 1];
    if (option == "--bind-address") {
      local.address = value;
    } else if (option == "--manual-bind-address") {
      manual_local.address = value;
    } else if (option == "--manual-bind-port") {
      manual_local.port = static_cast<std::uint16_t>(
          Number(value, std::numeric_limits<std::uint16_t>::max()));
    } else if (option == "--peer-address") {
      peer.address = value;
    } else if (option == "--bind-port") {
      local.port = static_cast<std::uint16_t>(
          Number(value, std::numeric_limits<std::uint16_t>::max()));
    } else if (option == "--peer-port") {
      peer.port = static_cast<std::uint16_t>(
          Number(value, std::numeric_limits<std::uint16_t>::max()));
    } else if (option == "--inbound-latency-ms") {
      config.network.inbound.latency = std::chrono::milliseconds{
          Number(value, std::numeric_limits<std::int32_t>::max())};
    } else if (option == "--outbound-latency-ms") {
      config.network.outbound.latency = std::chrono::milliseconds{
          Number(value, std::numeric_limits<std::int32_t>::max())};
    } else {
      throw std::invalid_argument("Unknown option: " + std::string(option));
    }
  }
  if (peer.port == 0) {
    throw std::invalid_argument("Peer port must be nonzero");
  }
  return config;
}

}  // namespace

int main(int argc, char** argv) try {
  const auto config = ReadOptions({argv, std::next(argv, argc)});
  if (!config) {
    return 0;
  }
  const auto& local = config->transport.local;
  const auto& manual_local = config->transport.manual_local;
  const auto& peer = config->transport.peer;
  DynamicModel model{config->dynamic_model.linear_speed_meters_per_second,
                     config->dynamic_model.angular_speed_radians_per_second};
  BatteryModel battery{config->battery.max_capacity_amp_hours,
                       config->battery.start_capacity_amp_hours};
  const auto start = std::chrono::steady_clock::now();
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start,
                                       config->state_machine.readiness_timeout};
  Controller controller{model, battery, state_machine, start};
  for (const auto& loss : config->controller.gnss_losses) {
    if (start > simulation::TimePoint::max() - loss.start) {
      throw std::invalid_argument(
          "GNSS loss start exceeds simulation clock range");
    }
    controller.AddGnssLoss(start + loss.start, loss.duration);
  }
  for (const auto& error : config->battery.battery_errors) {
    controller.AddBatteryError(start + simulation::Duration(10000), 0.3);
  }
  UdpTransport transport{local, peer};
  UdpTransport manual_transport{manual_local, peer};
  std::unique_ptr<SyslogSender> syslog;
  std::function<void(const Logger::Record&)> record_sink;
  if (config->syslog) {
    try {
      syslog = std::make_unique<SyslogSender>(*config->syslog, start);
      record_sink = [&syslog](const Logger::Record& record) {
        syslog->Send(record);
      };
    } catch (const std::exception& error) {
      std::cerr << "syslog disabled: " << error.what() << '\n';
    }
  }
  SimulationRunner runner{
      controller,
      transport,
      manual_transport,
      CreateNetworkEmulator(config->network.inbound, start),
      CreateNetworkEmulator(config->network.outbound, start),
      logger,
      std::move(record_sink)};
  if (std::signal(SIGINT, ShutdownSignal::Stop) == SIG_ERR ||
      std::signal(SIGTERM, ShutdownSignal::Stop) == SIG_ERR) {
    throw std::runtime_error("Cannot install shutdown signal handlers");
  }
  const auto bound = transport.LocalEndpoint();
  const auto manual_bound = manual_transport.LocalEndpoint();
  std::cout << "Automatic listening on " << bound.address << ':' << bound.port
            << "; manual listening on " << manual_bound.address << ':'
            << manual_bound.port << "; peer " << peer.address << ':'
            << peer.port << "; timestamps use steady_clock\n"
            << std::flush;
  runner.Run(ShutdownSignal::Requested);
} catch (const std::exception& error) {
  std::cerr << "pixhawk_simulator: " << error.what() << '\n';
  return 1;
}
