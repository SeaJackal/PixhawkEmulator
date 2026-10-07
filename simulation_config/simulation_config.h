#pragma once

#include <filesystem>
#include <optional>
#include <string_view>
#include <vector>

#include "network_emulator.h"
#include "udp_transport.h"

struct DynamicModelConfig {
  double linear_speed_meters_per_second = 1.0;
  double angular_speed_radians_per_second = 1.0;
};

struct BatteryConfig {
  double max_capacity_amp_hours = 2.0;
  double start_capacity_amp_hours = 2.0;
};

struct StateMachineConfig {
  simulation::Duration readiness_timeout = std::chrono::milliseconds{1000};
};

struct TransportConfig {
  UdpEndpoint local{"127.0.0.1", 14560};
  UdpEndpoint peer{"127.0.0.1", 14550};
};

struct GnssLossConfig {
  simulation::Duration start{};
  simulation::Duration duration{};
};

struct ControllerConfig {
  std::vector<GnssLossConfig> gnss_losses;
};

struct NetworkFilterConfig {
  enum class Kind { kDelay, kDrop };

  Kind kind = Kind::kDrop;
  MessageType message_type = MessageType::kHeartbeat;
  simulation::Duration start{};
  simulation::Duration end{};
  simulation::Duration delay{};
};

struct NetworkConfig {
  simulation::Duration latency{};
  std::vector<NetworkFilterConfig> filters;
};

struct SimulationConfig {
  DynamicModelConfig dynamic_model;
  BatteryConfig battery;
  StateMachineConfig state_machine;
  ControllerConfig controller;
  TransportConfig transport;
  std::optional<UdpEndpoint> syslog;
  struct Network {
    NetworkConfig inbound;
    NetworkConfig outbound;
  } network;
};

SimulationConfig ParseSimulationConfig(std::string_view json);
SimulationConfig LoadSimulationConfig(const std::filesystem::path& path);
NetworkEmulator CreateNetworkEmulator(const NetworkConfig& config,
                                      simulation::TimePoint start);
