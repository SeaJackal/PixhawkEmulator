#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string_view>
#include <vector>

#include "message_type.h"
#include "network_emulator.h"
#include "simulation_time.h"
#include "udp_transport.h"

struct DynamicModelConfig {
  double linear_speed_meters_per_second = 1.0;
  double angular_speed_radians_per_second = 1.0;
};

struct BatteryErrorConfig {
  simulation::Duration point{};
  double charge_diff{};
};

struct BatteryConfig {
  static constexpr double kDefaultCapacityAmpHours = 2.0;
  double max_capacity_amp_hours = kDefaultCapacityAmpHours;
  double start_capacity_amp_hours = kDefaultCapacityAmpHours;
  std::vector<BatteryErrorConfig> battery_errors;
};

struct StateMachineConfig {
  simulation::Duration readiness_timeout = std::chrono::seconds{1};
};

struct TransportConfig {
  static constexpr std::uint16_t kDefaultLocalPort = 14560;
  static constexpr std::uint16_t kDefaultManualPort = 14561;
  static constexpr std::uint16_t kDefaultPeerPort = 14550;
  UdpEndpoint local{"127.0.0.1", kDefaultLocalPort};
  UdpEndpoint manual_local{"127.0.0.1", kDefaultManualPort};
  UdpEndpoint peer{"127.0.0.1", kDefaultPeerPort};
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
