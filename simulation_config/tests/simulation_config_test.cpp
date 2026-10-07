#include "simulation_config.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace std::chrono_literals;
using TimePoint = simulation::TimePoint;
using Duration = simulation::Duration;

NetworkMessage MessageAt(TimePoint time,
                         MessageType type = MessageType::kHeartbeat) {
  auto message = NetworkMessage{
      HeartbeatMessage{time, ControllerStateMachine::State::kReady, true}};
  message.type = type;
  return message;
}

void ExpectInvalid(std::string_view json, std::string_view field) {
  try {
    ParseSimulationConfig(json);
    FAIL() << "Accepted invalid configuration: " << json;
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find(field), std::string::npos)
        << error.what();
  }
}

TEST(SimulationConfigTest, DefaultsMatchExistingSimulation) {
  const auto config = ParseSimulationConfig("{}");
  EXPECT_DOUBLE_EQ(config.dynamic_model.linear_speed_meters_per_second, 1.0);
  EXPECT_DOUBLE_EQ(config.dynamic_model.angular_speed_radians_per_second, 1.0);
  EXPECT_DOUBLE_EQ(config.battery.max_capacity_amp_hours, 2.0);
  EXPECT_DOUBLE_EQ(config.battery.start_capacity_amp_hours, 2.0);
  EXPECT_EQ(config.state_machine.readiness_timeout, 1s);
  EXPECT_EQ(config.transport.local.address, "127.0.0.1");
  EXPECT_EQ(config.transport.local.port, 14560);
  EXPECT_EQ(config.transport.peer.address, "127.0.0.1");
  EXPECT_EQ(config.transport.peer.port, 14550);
  EXPECT_EQ(config.network.inbound.latency, 0ms);
  EXPECT_EQ(config.network.outbound.latency, 0ms);
  EXPECT_TRUE(config.network.inbound.filters.empty());
  EXPECT_TRUE(config.network.outbound.filters.empty());
  EXPECT_FALSE(config.syslog.has_value());
  EXPECT_TRUE(config.controller.gnss_losses.empty());
}

TEST(SimulationConfigTest, ParsesGnssLossSchedules) {
  const auto config = ParseSimulationConfig(R"({"controller":{"gnss_losses":[
    {"start_ms":12000,"duration_ms":1000},
    {"start_ms":5000,"duration_ms":2000},
    {"start_ms":0,"duration_ms":0}
  ]}})");
  ASSERT_EQ(config.controller.gnss_losses.size(), 3u);
  EXPECT_EQ(config.controller.gnss_losses[0].start, 12s);
  EXPECT_EQ(config.controller.gnss_losses[0].duration, 1s);
  EXPECT_EQ(config.controller.gnss_losses[1].start, 5s);
  EXPECT_EQ(config.controller.gnss_losses[1].duration, 2s);
  EXPECT_EQ(config.controller.gnss_losses[2].start, 0ms);
  EXPECT_EQ(config.controller.gnss_losses[2].duration, 0ms);
  EXPECT_TRUE(ParseSimulationConfig(R"({"controller":{}})").controller.gnss_losses.empty());
  EXPECT_TRUE(ParseSimulationConfig(R"({"controller":{"gnss_losses":[]}})")
                  .controller.gnss_losses.empty());
}

TEST(SimulationConfigTest, RejectsMalformedGnssLossSchedules) {
  for (const auto json : {
           R"({"controller":null})",
           R"({"controller":{"unknown":[]}})",
           R"({"controller":{"gnss_losses":{}}})",
           R"({"controller":{"gnss_losses":[null]}})",
           R"({"controller":{"gnss_losses":[{"duration_ms":1}]}})",
           R"({"controller":{"gnss_losses":[{"start_ms":0}]}})",
           R"({"controller":{"gnss_losses":[{"start_ms":0,"duration_ms":1,"end_ms":1}]}})"}) {
    ExpectInvalid(json, "controller");
  }
  for (const auto field : {"start_ms", "duration_ms"}) {
    const auto other = std::string_view{field} == "start_ms" ? "duration_ms" : "start_ms";
    for (const auto value : {"-1", "1.5", "true", "null", "\"1\"", "[]", "{}",
                              "18446744073709551615"}) {
      const auto json = std::string{"{\"controller\":{\"gnss_losses\":[{\""} +
                        field + "\":" + value + ",\"" + other + "\":0}]}}";
      ExpectInvalid(json, std::string{"controller.gnss_losses[0]."} + field);
    }
  }
  const auto maximum = std::chrono::duration_cast<std::chrono::milliseconds>(
                           Duration::max()).count();
  const auto prefix = std::string{"{\"controller\":{\"gnss_losses\":[{\"start_ms\":"} +
                      std::to_string(maximum);
  EXPECT_NO_THROW(ParseSimulationConfig(prefix + ",\"duration_ms\":0}]}}"));
  ExpectInvalid(prefix + ",\"duration_ms\":1}]}}", "controller.gnss_losses[0].duration_ms");
}

TEST(SimulationConfigTest, ConfiguresOptionalSyslogCollector) {
  const auto defaults = ParseSimulationConfig(R"({"syslog":{}})");
  ASSERT_TRUE(defaults.syslog);
  EXPECT_EQ(defaults.syslog->address, "127.0.0.1");
  EXPECT_EQ(defaults.syslog->port, 514);
  const auto custom = ParseSimulationConfig(
      R"({"syslog":{"address":"192.0.2.1","port":5514}})");
  ASSERT_TRUE(custom.syslog);
  EXPECT_EQ(custom.syslog->address, "192.0.2.1");
  EXPECT_EQ(custom.syslog->port, 5514);
  const auto partial = ParseSimulationConfig(R"({"syslog":{"port":5514}})");
  ASSERT_TRUE(partial.syslog);
  EXPECT_EQ(partial.syslog->address, "127.0.0.1");
  EXPECT_EQ(partial.syslog->port, 5514);
  ExpectInvalid(R"({"syslog":null})", "syslog");
  ExpectInvalid(R"({"syslog":{"enabled":true}})", "syslog.enabled");
  ExpectInvalid(R"({"syslog":{"address":"localhost"}})", "syslog.address");
  ExpectInvalid(R"({"syslog":{"address":"127.0.0.1\u0000"}})", "syslog.address");
  ExpectInvalid(R"({"syslog":{"port":0}})", "syslog.port");
  ExpectInvalid(R"({"syslog":{"port":65536}})", "syslog.port");
  ExpectInvalid(R"({"syslog":{"port":-1}})", "syslog.port");
}

TEST(SimulationConfigTest, PartialOverridesKeepIndependentDefaults) {
  const auto config = ParseSimulationConfig(R"({
    "dynamic_model":{"linear_speed_meters_per_second":0},
    "battery":{"max_capacity_amp_hours":4},
    "transport":{"local":{"port":0}},
    "network":{"inbound":{"latency_ms":25}}
  })");
  EXPECT_DOUBLE_EQ(config.dynamic_model.linear_speed_meters_per_second, 0);
  EXPECT_DOUBLE_EQ(config.dynamic_model.angular_speed_radians_per_second, 1);
  EXPECT_DOUBLE_EQ(config.battery.max_capacity_amp_hours, 4);
  EXPECT_DOUBLE_EQ(config.battery.start_capacity_amp_hours, 2);
  EXPECT_EQ(config.transport.local.port, 0);
  EXPECT_EQ(config.transport.local.address, "127.0.0.1");
  EXPECT_EQ(config.network.inbound.latency, 25ms);
  EXPECT_EQ(config.network.outbound.latency, 0ms);
  ExpectInvalid(R"({"battery":{"max_capacity_amp_hours":1}})",
                "battery.start_capacity_amp_hours");
}

TEST(SimulationConfigTest, ParsesAllSimulationParameters) {
  const auto config = ParseSimulationConfig(R"({
    "dynamic_model":{
      "linear_speed_meters_per_second":3.5,
      "angular_speed_radians_per_second":2
    },
    "battery":{"max_capacity_amp_hours":5,"start_capacity_amp_hours":0},
    "state_machine":{"readiness_timeout_ms":0},
    "transport":{
      "local":{"address":"0.0.0.0","port":65535},
      "peer":{"address":"192.168.1.2","port":1}
    },
    "network":{"inbound":{},"outbound":{"latency_ms":100,"filters":[]}}
  })");
  EXPECT_DOUBLE_EQ(config.dynamic_model.linear_speed_meters_per_second, 3.5);
  EXPECT_DOUBLE_EQ(config.dynamic_model.angular_speed_radians_per_second, 2);
  EXPECT_DOUBLE_EQ(config.battery.max_capacity_amp_hours, 5);
  EXPECT_DOUBLE_EQ(config.battery.start_capacity_amp_hours, 0);
  EXPECT_EQ(config.state_machine.readiness_timeout, 0ms);
  EXPECT_EQ(config.transport.local.address, "0.0.0.0");
  EXPECT_EQ(config.transport.local.port, 65535);
  EXPECT_EQ(config.transport.peer.address, "192.168.1.2");
  EXPECT_EQ(config.transport.peer.port, 1);
  EXPECT_EQ(config.network.outbound.latency, 100ms);
}

TEST(SimulationConfigTest, LoadsCompleteExampleAndAppliesIndependentTroubles) {
  const auto config = LoadSimulationConfig(SIMULATION_CONFIG_EXAMPLE);
  ASSERT_EQ(config.controller.gnss_losses.size(), 2u);
  EXPECT_EQ(config.controller.gnss_losses[0].start, 5s);
  EXPECT_EQ(config.controller.gnss_losses[0].duration, 2s);
  EXPECT_EQ(config.controller.gnss_losses[1].start, 12s);
  EXPECT_EQ(config.controller.gnss_losses[1].duration, 1s);
  ASSERT_EQ(config.network.inbound.filters.size(), 1u);
  ASSERT_EQ(config.network.outbound.filters.size(), 1u);
  const TimePoint start{42s};
  const auto inbound = CreateNetworkEmulator(config.network.inbound, start);
  const auto outbound = CreateNetworkEmulator(config.network.outbound, start);
  for (const auto offset : {9999ms, 10000ms, 14999ms, 15000ms}) {
    const auto message =
        MessageAt(start + offset, MessageType::kAutomaticControl);
    const auto received = inbound.Process(message);
    if (offset >= 10000ms && offset < 15000ms)
      EXPECT_FALSE(received);
    else {
      ASSERT_TRUE(received);
      EXPECT_EQ(received->arrival_time, message.timestamp + 20ms);
    }
    const auto sent = outbound.Process(message);
    ASSERT_TRUE(sent);
    EXPECT_EQ(sent->arrival_time, message.timestamp + 50ms);
  }
  for (const auto offset : {9999ms, 10000ms, 14999ms, 15000ms}) {
    const auto message = MessageAt(start + offset, MessageType::kAttitude);
    const auto sent = outbound.Process(message);
    ASSERT_TRUE(sent);
    const auto extra = offset >= 10000ms && offset < 15000ms ? 200ms : 0ms;
    EXPECT_EQ(sent->arrival_time, message.timestamp + 50ms + extra);
    const auto received = inbound.Process(message);
    ASSERT_TRUE(received);
    EXPECT_EQ(received->arrival_time, message.timestamp + 20ms);
  }
}

TEST(SimulationConfigTest, SupportsEveryMessageType) {
  const std::array types{
      std::pair{"heartbeat", MessageType::kHeartbeat},
      std::pair{"attitude", MessageType::kAttitude},
      std::pair{"global_position", MessageType::kGlobalPosition},
      std::pair{"battery_status", MessageType::kBatteryStatus},
      std::pair{"command", MessageType::kCommand},
      std::pair{"automatic_control", MessageType::kAutomaticControl},
      std::pair{"manual_control", MessageType::kManualControl}};
  for (const auto& [name, type] : types) {
    SCOPED_TRACE(name);
    const auto config = ParseSimulationConfig(
        std::string{
            R"({"network":{"inbound":{"filters":[{"kind":"drop","message_type":")"} +
        name + R"(","start_ms":0,"end_ms":1000}]}}})");
    ASSERT_EQ(config.network.inbound.filters.size(), 1u);
    EXPECT_EQ(config.network.inbound.filters[0].message_type, type);
    const auto emulator =
        CreateNetworkEmulator(config.network.inbound, TimePoint{10s});
    EXPECT_FALSE(emulator.Process(MessageAt(TimePoint{10500ms}, type)));
  }
}

TEST(SimulationConfigTest, PreservesFirstMatchingFilterOrder) {
  auto config = ParseSimulationConfig(R"({"network":{"inbound":{
    "latency_ms":10,
    "filters":[
      {"kind":"delay","message_type":"heartbeat","start_ms":0,"end_ms":1000,"delay_ms":50},
      {"kind":"drop","message_type":"heartbeat","start_ms":0,"end_ms":1000}
    ]
  }}})");
  const TimePoint start{10s};
  auto emulator = CreateNetworkEmulator(config.network.inbound, start);
  const auto result = emulator.Process(MessageAt(start));
  ASSERT_TRUE(result);
  EXPECT_EQ(result->arrival_time, start + 60ms);
  std::swap(config.network.inbound.filters[0],
            config.network.inbound.filters[1]);
  emulator = CreateNetworkEmulator(config.network.inbound, start);
  EXPECT_FALSE(emulator.Process(MessageAt(start)));
}

TEST(SimulationConfigTest, RejectsInvalidJsonStructureAndValues) {
  const std::array cases{
      std::pair{"", "$"},
      std::pair{"{", "$"},
      std::pair{"{} trailing", "$"},
      std::pair{"[]", "$"},
      std::pair{"null", "$"},
      std::pair{R"({"unknown":0})", "unknown"},
      std::pair{R"({"dynamic_model":null})", "dynamic_model"},
      std::pair{R"({"battery":[]})", "battery"},
      std::pair{R"({"state_machine":false})", "state_machine"},
      std::pair{R"({"transport":1})", "transport"},
      std::pair{R"({"network":[]})", "network"},
      std::pair{R"({"dynamic_model":{"speed":1}})", "dynamic_model.speed"},
      std::pair{R"({"dynamic_model":{"linear_speed_meters_per_second":-1}})",
                "linear_speed"},
      std::pair{R"({"dynamic_model":{"angular_speed_radians_per_second":"1"}})",
                "angular_speed"},
      std::pair{
          R"({"dynamic_model":{"angular_speed_radians_per_second":true}})",
          "angular_speed"},
      std::pair{
          R"({"dynamic_model":{"angular_speed_radians_per_second":1e999}})",
          "angular_speed"},
      std::pair{R"({"battery":{"max_capacity_amp_hours":0}})", "max_capacity"},
      std::pair{R"({"battery":{"start_capacity_amp_hours":-1}})",
                "start_capacity"},
      std::pair{R"({"battery":{"start_capacity_amp_hours":3}})",
                "start_capacity"},
      std::pair{R"({"state_machine":{"readiness_timeout_ms":1.5}})",
                "readiness_timeout"},
      std::pair{R"({"transport":{"local":{"address":"::1"}}})",
                "local.address"},
      std::pair{R"({"transport":{"peer":{"address":"localhost"}}})",
                "peer.address"},
      std::pair{R"({"transport":{"peer":{"address":"127.0.0.1\u0000bad"}}})",
                "peer.address"},
      std::pair{R"({"transport":{"peer":{"port":0}}})", "peer.port"},
      std::pair{R"({"transport":{"local":{"port":65536}}})", "local.port"},
      std::pair{R"({"transport":{"local":{"port":-1}}})", "local.port"},
      std::pair{R"({"transport":{"local":{"port":1.0}}})", "local.port"},
      std::pair{R"({"network":{"inbound":{"latency_ms":-1}}})",
                "inbound.latency_ms"},
      std::pair{R"({"network":{"inbound":{"latency_ms":null}}})",
                "inbound.latency_ms"},
      std::pair{R"({"network":{"outbound":{"filters":{}}}})",
                "outbound.filters"},
      std::pair{R"({"network":{"outbound":{"unknown":1}}})",
                "outbound.unknown"}};
  for (const auto& [json, field] : cases) {
    SCOPED_TRACE(json);
    ExpectInvalid(json, field);
  }
}

TEST(SimulationConfigTest, RejectsInvalidFilterFieldsWithIndexedPaths) {
  const std::array cases{
      std::pair{R"(null)", "filters[0]"},
      std::pair{R"({})", "kind"},
      std::pair{R"({"kind":"other"})", "kind"},
      std::pair{R"({"kind":"drop"})", "message_type"},
      std::pair{R"({"kind":"drop","message_type":"unknown"})", "message_type"},
      std::pair{R"({"kind":"drop","message_type":"heartbeat"})", "start_ms"},
      std::pair{R"({"kind":"drop","message_type":"heartbeat","start_ms":0})",
                "end_ms"},
      std::pair{
          R"({"kind":"drop","message_type":"heartbeat","start_ms":2,"end_ms":1})",
          "end_ms"},
      std::pair{
          R"({"kind":"delay","message_type":"heartbeat","start_ms":0,"end_ms":1})",
          "delay_ms"},
      std::pair{
          R"({"kind":"drop","message_type":"heartbeat","start_ms":0,"end_ms":1,"delay_ms":0})",
          "delay_ms"},
      std::pair{
          R"({"kind":"delay","message_type":"heartbeat","start_ms":0,"end_ms":1,"delay_ms":-1})",
          "delay_ms"},
      std::pair{
          R"({"kind":"drop","message_type":"heartbeat","start_ms":0,"end_ms":1,"extra":0})",
          "extra"}};
  for (const auto& [filter, field] : cases) {
    const auto json =
        std::string{R"({"network":{"inbound":{"filters":[)"} + filter + "]}}}";
    ExpectInvalid(json, "network.inbound.filters[0]");
    ExpectInvalid(json, field);
  }
}

TEST(SimulationConfigTest, ChecksDurationConversionBoundaries) {
  const auto maximum =
      std::chrono::duration_cast<std::chrono::milliseconds>(Duration::max());
  const auto json = [](std::uint64_t milliseconds) {
    return "{\"state_machine\":{\"readiness_timeout_ms\":" +
           std::to_string(milliseconds) + "}}";
  };
  EXPECT_EQ(ParseSimulationConfig(json(maximum.count()))
                .state_machine.readiness_timeout,
            std::chrono::duration_cast<Duration>(maximum));
  ExpectInvalid(json(static_cast<std::uint64_t>(maximum.count()) + 1),
                "readiness_timeout_ms");
  ExpectInvalid(json(std::numeric_limits<std::uint64_t>::max()),
                "readiness_timeout_ms");
}

TEST(SimulationConfigTest, FactoryChecksTimestampOverflowAndEmptyWindows) {
  const auto config =
      ParseSimulationConfig(R"({"network":{"inbound":{"filters":[
    {"kind":"drop","message_type":"heartbeat","start_ms":0,"end_ms":1}
  ]}}})");
  EXPECT_THROW(CreateNetworkEmulator(config.network.inbound, TimePoint::max()),
               std::invalid_argument);
  EXPECT_NO_THROW(
      CreateNetworkEmulator(config.network.inbound, TimePoint::max() - 1ms));
  EXPECT_NO_THROW(
      CreateNetworkEmulator(config.network.inbound, TimePoint::min()));
  auto empty = config.network.inbound;
  empty.filters[0].end = 0ms;
  const auto emulator = CreateNetworkEmulator(empty, TimePoint{10s});
  EXPECT_TRUE(emulator.Process(MessageAt(TimePoint{10s})));
  empty.latency = -1ms;
  EXPECT_THROW(CreateNetworkEmulator(empty, TimePoint{}),
               std::invalid_argument);
  empty.latency = 0ms;
  empty.filters[0].start = -1ms;
  EXPECT_THROW(CreateNetworkEmulator(empty, TimePoint{}),
               std::invalid_argument);
}

TEST(SimulationConfigTest, FileFailuresIncludeFilename) {
  const auto example = std::filesystem::path{SIMULATION_CONFIG_EXAMPLE};
  for (const auto& path : {example / "missing.json", example.parent_path()}) {
    try {
      LoadSimulationConfig(path);
      FAIL() << "Accepted unreadable file";
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string{error.what()}.find(path.string()),
                std::string::npos);
    }
  }
}

}
