#include "simulation_config.h"

#include <algorithm>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/json/object.hpp>
#include <boost/json/parse.hpp>
#include <boost/json/value.hpp>
#include <boost/system/error_code.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include "message_type.h"
#include "network_emulator.h"
#include "simulation_time.h"
#include "udp_transport.h"

namespace {

using Object = boost::json::object;
using Value = boost::json::value;
using Duration = simulation::Duration;
using TimePoint = simulation::TimePoint;

[[noreturn]] void Invalid(const std::string& path, const std::string& reason);

const Object& ReadObject(const Value& value, const std::string& path,
                         std::initializer_list<std::string_view> keys);

const Value& Required(const Object& object, const char* key,
                      const std::string& path);

std::string ReadString(const Value& value, const std::string& path);

std::uint64_t ReadUnsigned(const Value& value, const std::string& path);

double ReadNumber(const Value& value, const std::string& path);

Duration ReadDuration(const Value& value, const std::string& path);

MessageType ReadMessageType(const Value& value, const std::string& path);

void ReadEndpoint(const Value& value, const std::string& path,
                  UdpEndpoint& endpoint, bool peer);

NetworkConfig ReadNetwork(const Value& value, const std::string& path);

ControllerConfig ReadController(const Value& value);

BatteryConfig ReadBattery(const Value& value);

TimePoint AddOffset(TimePoint start, Duration offset, const std::string& path);

// -----------------------------------------------------------------------------

[[noreturn]] void Invalid(const std::string& path, const std::string& reason) {
  throw std::invalid_argument(path + ": " + reason);
}

const Object& ReadObject(const Value& value, const std::string& path,
                         std::initializer_list<std::string_view> keys) {
  if (!value.is_object()) {
    Invalid(path, "expected object");
  }
  const auto& object = value.as_object();
  for (const auto& entry : object) {
    const std::string_view key{entry.key().data(), entry.key().size()};
    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
      Invalid(path + "." + std::string(key), "unknown field");
    }
  }
  return object;
}

const Value& Required(const Object& object, const char* key,
                      const std::string& path) {
  const auto* value = object.if_contains(key);
  if (value == nullptr) {
    Invalid(path + "." + key, "required field");
  }
  return *value;
}

std::string ReadString(const Value& value, const std::string& path) {
  if (!value.is_string()) {
    Invalid(path, "expected string");
  }
  const auto& text = value.as_string();
  return {text.data(), text.size()};
}

std::uint64_t ReadUnsigned(const Value& value, const std::string& path) {
  if (value.is_uint64()) {
    return value.as_uint64();
  }
  if (value.is_int64() && value.as_int64() >= 0) {
    return static_cast<std::uint64_t>(value.as_int64());
  }
  Invalid(path, "expected nonnegative integer");
}

double ReadNumber(const Value& value, const std::string& path) {
  double number = 0.0;
  if (value.is_double()) {
    number = value.as_double();
  } else if (value.is_int64()) {
    number = static_cast<double>(value.as_int64());
  } else if (value.is_uint64()) {
    number = static_cast<double>(value.as_uint64());
  } else {
    Invalid(path, "expected number");
  }
  if (!std::isfinite(number) || number < 0.0) {
    Invalid(path, "expected finite nonnegative number");
  }
  return number;
}

Duration ReadDuration(const Value& value, const std::string& path) {
  const auto milliseconds = ReadUnsigned(value, path);
  const auto maximum =
      std::chrono::duration_cast<std::chrono::milliseconds>(Duration::max())
          .count();
  if (milliseconds > static_cast<std::uint64_t>(maximum)) {
    Invalid(path, "duration exceeds simulation clock range");
  }
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds{static_cast<std::int64_t>(milliseconds)});
}

MessageType ReadMessageType(const Value& value, const std::string& path) {
  const auto name = ReadString(value, path);
  if (name == "heartbeat") {
    return MessageType::kHeartbeat;
  }
  if (name == "attitude") {
    return MessageType::kAttitude;
  }
  if (name == "global_position") {
    return MessageType::kGlobalPosition;
  }
  if (name == "battery_status") {
    return MessageType::kBatteryStatus;
  }
  if (name == "command") {
    return MessageType::kCommand;
  }
  if (name == "control") {
    return MessageType::kControl;
  }
  Invalid(path, "unknown message type");
}

void ReadEndpoint(const Value& value, const std::string& path,
                  UdpEndpoint& endpoint, bool peer) {
  const auto& object = ReadObject(value, path, {"address", "port"});
  if (const auto* address = object.if_contains("address")) {
    endpoint.address = ReadString(*address, path + ".address");
    boost::system::error_code error;
    boost::asio::ip::make_address_v4(endpoint.address, error);
    if (error || endpoint.address.find('\0') != std::string::npos) {
      Invalid(path + ".address", "expected IPv4 address");
    }
  }
  if (const auto* port = object.if_contains("port")) {
    const auto number = ReadUnsigned(*port, path + ".port");
    if (number > std::numeric_limits<std::uint16_t>::max() ||
        (peer && number == 0)) {
      Invalid(path + ".port",
              peer ? "expected port 1..65535" : "expected port 0..65535");
    }
    endpoint.port = static_cast<std::uint16_t>(number);
  }
}

NetworkConfig ReadNetwork(const Value& value, const std::string& path) {
  NetworkConfig config;
  const auto& object = ReadObject(value, path, {"latency_ms", "filters"});
  if (const auto* latency = object.if_contains("latency_ms")) {
    config.latency = ReadDuration(*latency, path + ".latency_ms");
  }
  if (const auto* filters = object.if_contains("filters")) {
    if (!filters->is_array()) {
      Invalid(path + ".filters", "expected array");
    }
    for (const auto& value : filters->as_array()) {
      const auto filter_path =
          path + ".filters[" + std::to_string(config.filters.size()) + "]";
      const auto& filter = ReadObject(
          value, filter_path,
          {"kind", "message_type", "start_ms", "end_ms", "delay_ms"});
      NetworkFilterConfig result;
      const auto kind = ReadString(Required(filter, "kind", filter_path),
                                   filter_path + ".kind");
      if (kind == "delay") {
        result.kind = NetworkFilterConfig::Kind::kDelay;
      } else if (kind == "drop") {
        result.kind = NetworkFilterConfig::Kind::kDrop;
      } else {
        Invalid(filter_path + ".kind", "expected delay or drop");
      }
      result.message_type =
          ReadMessageType(Required(filter, "message_type", filter_path),
                          filter_path + ".message_type");
      result.start = ReadDuration(Required(filter, "start_ms", filter_path),
                                  filter_path + ".start_ms");
      result.end = ReadDuration(Required(filter, "end_ms", filter_path),
                                filter_path + ".end_ms");
      if (result.start > result.end) {
        Invalid(filter_path + ".end_ms", "must be at least start_ms");
      }
      if (result.kind == NetworkFilterConfig::Kind::kDelay) {
        result.delay = ReadDuration(Required(filter, "delay_ms", filter_path),
                                    filter_path + ".delay_ms");
      } else if (filter.if_contains("delay_ms") != nullptr) {
        Invalid(filter_path + ".delay_ms", "not allowed for drop filters");
      }
      config.filters.push_back(result);
    }
  }
  return config;
}

ControllerConfig ReadController(const Value& value) {
  ControllerConfig config;
  const auto& object = ReadObject(value, "controller", {"gnss_losses"});
  if (const auto* losses = object.if_contains("gnss_losses")) {
    if (!losses->is_array()) {
      Invalid("controller.gnss_losses", "expected array");
    }
    for (const auto& value : losses->as_array()) {
      const auto path = "controller.gnss_losses[" +
                        std::to_string(config.gnss_losses.size()) + "]";
      const auto& loss = ReadObject(value, path, {"start_ms", "duration_ms"});
      GnssLossConfig result;
      result.start =
          ReadDuration(Required(loss, "start_ms", path), path + ".start_ms");
      result.duration = ReadDuration(Required(loss, "duration_ms", path),
                                     path + ".duration_ms");
      if (result.start > Duration::max() - result.duration) {
        Invalid(path + ".duration_ms",
                "loss end exceeds simulation clock range");
      }
      config.gnss_losses.push_back(result);
    }
  }

  return config;
}

BatteryConfig ReadBattery(const Value& value) {
  BatteryConfig config;
  const auto& object = ReadObject(
      value, "battery", {"max_capacity_amp_hours", "start_capacity_amp_hours"});
  if (const auto* capacity = object.if_contains("max_capacity_amp_hours")) {
    config.max_capacity_amp_hours =
        ReadNumber(*capacity, "battery.max_capacity_amp_hours");
  }
  if (const auto* capacity = object.if_contains("start_capacity_amp_hours")) {
    config.start_capacity_amp_hours =
        ReadNumber(*capacity, "battery.start_capacity_amp_hours");
  }
  if (config.max_capacity_amp_hours <= 0.0) {
    Invalid("battery.max_capacity_amp_hours", "must be positive");
  }
  if (config.start_capacity_amp_hours > config.max_capacity_amp_hours) {
    Invalid("battery.start_capacity_amp_hours", "exceeds maximum capacity");
  }

  return config;
}

TimePoint AddOffset(TimePoint start, Duration offset, const std::string& path) {
  if (offset < Duration::zero() || start > TimePoint::max() - offset) {
    Invalid(path, "timestamp exceeds simulation clock range");
  }
  return start + offset;
}

}

SimulationConfig ParseSimulationConfig(std::string_view json) {
  boost::system::error_code error;
  const auto value = boost::json::parse(
      boost::json::string_view{json.data(), json.size()}, error);
  if (error) {
    Invalid("$", "invalid JSON: " + error.message());
  }
  const auto& root =
      ReadObject(value, "$",
                 {"dynamic_model", "battery", "state_machine", "controller",
                  "transport", "network", "syslog"});
  SimulationConfig config;
  if (const auto* controller = root.if_contains("controller")) {
    config.controller = ReadController(*controller);
  }
  if (const auto* syslog = root.if_contains("syslog")) {
    constexpr std::uint16_t kSyslogPort = 514;
    config.syslog = UdpEndpoint{"127.0.0.1", kSyslogPort};
    ReadEndpoint(*syslog, "syslog", *config.syslog, true);
  }
  if (const auto* model = root.if_contains("dynamic_model")) {
    const auto& object = ReadObject(
        *model, "dynamic_model",
        {"linear_speed_meters_per_second", "angular_speed_radians_per_second"});
    if (const auto* speed =
            object.if_contains("linear_speed_meters_per_second")) {
      config.dynamic_model.linear_speed_meters_per_second =
          ReadNumber(*speed, "dynamic_model.linear_speed_meters_per_second");
    }
    if (const auto* speed =
            object.if_contains("angular_speed_radians_per_second")) {
      config.dynamic_model.angular_speed_radians_per_second =
          ReadNumber(*speed, "dynamic_model.angular_speed_radians_per_second");
    }
  }
  if (const auto* battery = root.if_contains("battery")) {
    config.battery = ReadBattery(*battery);
  }
  if (const auto* state_machine = root.if_contains("state_machine")) {
    const auto& object =
        ReadObject(*state_machine, "state_machine", {"readiness_timeout_ms"});
    if (const auto* timeout = object.if_contains("readiness_timeout_ms")) {
      config.state_machine.readiness_timeout =
          ReadDuration(*timeout, "state_machine.readiness_timeout_ms");
    }
  }
  if (const auto* transport = root.if_contains("transport")) {
    const auto& object =
        ReadObject(*transport, "transport", {"local", "manual_local", "peer"});
    if (const auto* local = object.if_contains("local")) {
      ReadEndpoint(*local, "transport.local", config.transport.local, false);
    }
    if (const auto* manual = object.if_contains("manual_local")) {
      ReadEndpoint(*manual, "transport.manual_local",
                   config.transport.manual_local, false);
    }
    if (const auto* peer = object.if_contains("peer")) {
      ReadEndpoint(*peer, "transport.peer", config.transport.peer, true);
    }
  }
  if (const auto* network = root.if_contains("network")) {
    const auto& object =
        ReadObject(*network, "network", {"inbound", "outbound"});
    if (const auto* inbound = object.if_contains("inbound")) {
      config.network.inbound = ReadNetwork(*inbound, "network.inbound");
    }
    if (const auto* outbound = object.if_contains("outbound")) {
      config.network.outbound = ReadNetwork(*outbound, "network.outbound");
    }
  }
  return config;
}

SimulationConfig LoadSimulationConfig(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Cannot read config file: " + path.string());
  }
  std::string json;
  try {
    json.assign(std::istreambuf_iterator<char>{file},
                std::istreambuf_iterator<char>{});
  } catch (const std::ios_base::failure&) {
    throw std::runtime_error("Cannot read config file: " + path.string());
  }
  if (file.bad()) {
    throw std::runtime_error("Cannot read config file: " + path.string());
  }
  return ParseSimulationConfig(json);
}

NetworkEmulator CreateNetworkEmulator(const NetworkConfig& config,
                                      TimePoint start) {
  if (config.latency < Duration::zero()) {
    Invalid("network.latency", "must be nonnegative");
  }
  NetworkEmulator emulator{config.latency};
  for (std::size_t index = 0; index < config.filters.size(); ++index) {
    const auto& filter = config.filters[index];
    const auto path = "network.filters[" + std::to_string(index) + "]";
    if (filter.start > filter.end) {
      Invalid(path + ".end", "must be at least start");
    }
    if (filter.delay < Duration::zero()) {
      Invalid(path + ".delay", "must be nonnegative");
    }
    const auto begin = AddOffset(start, filter.start, path + ".start");
    const auto end = AddOffset(start, filter.end, path + ".end");
    switch (filter.kind) {
      case NetworkFilterConfig::Kind::kDelay:
        emulator.AddDelayFilter(begin, end, filter.message_type, filter.delay);
        break;
      case NetworkFilterConfig::Kind::kDrop:
        emulator.AddDropFilter(begin, end, filter.message_type);
        break;
      default:
        Invalid(path + ".kind", "expected delay or drop");
    }
  }
  return emulator;
}
