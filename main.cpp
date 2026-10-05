#include "controller.h"
#include "simulation_runner.h"
#include "udp_transport.h"

#include <charconv>
#include <csignal>
#include <iostream>
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

int main(int argc, char** argv) {
  UdpEndpoint local{"127.0.0.1", 14560};
  UdpEndpoint peer{"127.0.0.1", 14550};
  simulation::Duration inbound_latency{};
  simulation::Duration outbound_latency{};
  for (int index = 1; index < argc; ++index) {
    const std::string_view option = argv[index];
    if (option == "--help") {
      std::cout << "pixhawk_simulator [--bind-address IPv4] [--bind-port PORT]\n"
                   "                  [--peer-address IPv4] [--peer-port PORT]\n"
                   "                  [--inbound-latency-ms MS] [--outbound-latency-ms MS]\n";
      return 0;
    }
    if (++index == argc) throw std::invalid_argument("Missing option value");
    const std::string_view value = argv[index];
    if (option == "--bind-address") local.address = value;
    else if (option == "--peer-address") peer.address = value;
    else if (option == "--bind-port") local.port = Number(value, 65535);
    else if (option == "--peer-port") peer.port = Number(value, 65535);
    else if (option == "--inbound-latency-ms") {
      inbound_latency = std::chrono::milliseconds{Number(value, 2147483647)};
    } else if (option == "--outbound-latency-ms") {
      outbound_latency = std::chrono::milliseconds{Number(value, 2147483647)};
    } else {
      throw std::invalid_argument("Unknown option: " + std::string(option));
    }
  }
  if (peer.port == 0) throw std::invalid_argument("Peer port must be nonzero");
  DynamicModel model;
  BatteryModel battery;
  std::chrono::steady_clock clock;
  const auto start = clock.now();
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start};
  Controller controller{model, battery, state_machine, start};
  UdpTransport transport{local, peer};
  SimulationRunner runner{controller, transport, clock,
                          ScheduledLink{NetworkEmulator{inbound_latency}},
                          ScheduledLink{NetworkEmulator{outbound_latency}}};
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  const auto bound = transport.LocalEndpoint();
  std::cout << "Listening on " << bound.address << ':' << bound.port
            << "; peer " << peer.address << ':' << peer.port
            << "; timestamps use steady_clock\n" << std::flush;
  runner.Run([] { return stop_requested != 0; });
}
