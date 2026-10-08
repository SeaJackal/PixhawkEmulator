#include "telemetry_serialization.h"
#include "network_message.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace {

using namespace std::chrono_literals;
using State = ControllerStateMachine::State;

std::vector<std::byte> Bytes(std::initializer_list<unsigned char> values) {
  std::vector<std::byte> bytes;
  for (const auto value : values) {
    bytes.push_back(static_cast<std::byte>(value));
  }
  return bytes;
}

std::vector<std::byte> WithTimestamp(std::vector<std::byte> bytes) {
  const auto timestamp = Bytes({0x10, 0xa4, 0, 0, 0, 0, 0, 0});
  bytes.insert(bytes.end(), timestamp.begin(), timestamp.end());
  return bytes;
}

const auto kSixDoubles = Bytes({
    0, 0, 0, 0, 0, 0, 0xf0, 0x3f,
    0, 0, 0, 0, 0, 0, 0, 0xc0,
    0, 0, 0, 0, 0, 0, 0x0c, 0x40,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0x80,
    0, 0, 0, 0, 0, 0, 0xe0, 0x3f});

TEST(TelemetrySerializationTest, HeartbeatEncodesAllStatesAndReadiness) {
  const std::array states{State::kGuarded, State::kNotReady, State::kReady,
                          State::kManual, State::kAutomatic, State::kHolding};
  for (std::size_t index = 0; index < states.size(); ++index) {
    for (const bool ready : {false, true}) {
      const HeartbeatMessage message{simulation::TimePoint{42s}, states[index],
                                     ready};
      EXPECT_EQ(Serialize(message),
                WithTimestamp(Bytes({static_cast<unsigned char>(index),
                       static_cast<unsigned char>(ready)})));
      const auto decoded = Deserialize<HeartbeatMessage>(
          WithTimestamp(Bytes({static_cast<unsigned char>(index),
                 static_cast<unsigned char>(ready)})));
      EXPECT_EQ(decoded.state, message.state);
      EXPECT_EQ(decoded.ready, message.ready);
      EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
    }
  }
}

TEST(TelemetrySerializationTest, RejectsUnknownHeartbeatState) {
  const HeartbeatMessage message{{}, static_cast<State>(99), false};
  EXPECT_THROW(Serialize(message), std::invalid_argument);
}

TEST(TelemetrySerializationTest, AttitudeEncodesSixLittleEndianDoublesInFieldOrder) {
  const AttitudeMessage message{simulation::TimePoint{42s},
                                {1.0, -2.0, 3.5}, {0.0, -0.0, 0.5}};
  EXPECT_EQ(Serialize(message), WithTimestamp(kSixDoubles));
  const auto decoded = Deserialize<AttitudeMessage>(WithTimestamp(kSixDoubles));
  EXPECT_DOUBLE_EQ(decoded.orientation.roll_radians, 1.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.pitch_radians, -2.0);
  EXPECT_DOUBLE_EQ(decoded.orientation.yaw_radians, 3.5);
  EXPECT_DOUBLE_EQ(decoded.angular_velocity.roll_radians_per_second, 0.0);
  EXPECT_FALSE(std::signbit(decoded.angular_velocity.roll_radians_per_second));
  EXPECT_DOUBLE_EQ(decoded.angular_velocity.pitch_radians_per_second, -0.0);
  EXPECT_TRUE(std::signbit(decoded.angular_velocity.pitch_radians_per_second));
  EXPECT_DOUBLE_EQ(decoded.angular_velocity.yaw_radians_per_second, 0.5);
  EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
}

TEST(TelemetrySerializationTest, PositionEncodesSixLittleEndianDoublesInFieldOrder) {
  const GlobalPositionMessage message{simulation::TimePoint{42s},
                                      {1.0, -2.0, 3.5}, {0.0, -0.0, 0.5}};
  auto fields = kSixDoubles;
  fields.push_back(std::byte{1});
  EXPECT_EQ(Serialize(message), WithTimestamp(fields));
  const auto decoded = Deserialize<GlobalPositionMessage>(WithTimestamp(fields));
  EXPECT_TRUE(decoded.gnss_valid);
  EXPECT_DOUBLE_EQ(decoded.position.x_meters, 1.0);
  EXPECT_DOUBLE_EQ(decoded.position.y_meters, -2.0);
  EXPECT_DOUBLE_EQ(decoded.position.z_meters, 3.5);
  EXPECT_DOUBLE_EQ(decoded.velocity.x_meters_per_second, 0.0);
  EXPECT_FALSE(std::signbit(decoded.velocity.x_meters_per_second));
  EXPECT_DOUBLE_EQ(decoded.velocity.y_meters_per_second, -0.0);
  EXPECT_TRUE(std::signbit(decoded.velocity.y_meters_per_second));
  EXPECT_DOUBLE_EQ(decoded.velocity.z_meters_per_second, 0.5);
  EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
}

TEST(TelemetrySerializationTest, BatteryEncodesVoltageCurrentAndCharge) {
  const BatteryStatusMessage message{simulation::TimePoint{42s}, 12.5, -2.0, 0.5};
  EXPECT_EQ(Serialize(message), WithTimestamp(Bytes({
      0, 0, 0, 0, 0, 0, 0x29, 0x40,
      0, 0, 0, 0, 0, 0, 0, 0xc0,
      0, 0, 0, 0, 0, 0, 0xe0, 0x3f})));
  const auto decoded = Deserialize<BatteryStatusMessage>(Serialize(message));
  EXPECT_DOUBLE_EQ(decoded.voltage_volts, 12.5);
  EXPECT_DOUBLE_EQ(decoded.current_amps, -2.0);
  EXPECT_DOUBLE_EQ(decoded.charge_amp_hours, 0.5);
  EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
}

TEST(TelemetrySerializationTest, GnssValiditySurvivesPacketsAndAggregateTelemetry) {
  for (const bool valid : {false, true}) {
    const GlobalPositionMessage message{simulation::TimePoint{42s},
                                        {1.0, -2.0, 3.5}, {0.0, -0.0, 0.5}, valid};
    auto fields = kSixDoubles;
    fields.push_back(valid ? std::byte{1} : std::byte{0});
    const auto expected = WithTimestamp(fields);
    EXPECT_EQ(Serialize(message), expected);
    EXPECT_EQ(expected.size(), 57u);
    EXPECT_EQ(Deserialize<GlobalPositionMessage>(expected).gnss_valid, valid);
    auto packet_bytes = expected;
    packet_bytes.push_back(std::byte{2});
    const NetworkMessage packet{packet_bytes};
    EXPECT_EQ(packet.type, MessageType::kGlobalPosition);
    EXPECT_EQ(packet.timestamp, message.timestamp);
    EXPECT_EQ(NetworkMessage{message}.data, packet_bytes);
    Telemetry telemetry;
    telemetry.global_position = message;
    telemetry.battery_status = BatteryStatusMessage{message.timestamp, 12.0, 1.0, 2.0};
    std::vector<std::byte> aggregate{std::byte{0x0c}};
    aggregate.insert(aggregate.end(), expected.begin(), expected.end());
    const auto battery = Serialize(*telemetry.battery_status);
    aggregate.insert(aggregate.end(), battery.begin(), battery.end());
    EXPECT_EQ(Serialize(telemetry), aggregate);
  }
}

TEST(TelemetryDeserializationTest, RejectsInvalidGnssValidityFlags) {
  auto bytes = Serialize(GlobalPositionMessage{{}, {}, {}});
  for (const auto flag : {2, 255}) {
    bytes[48] = static_cast<std::byte>(flag);
    EXPECT_THROW(Deserialize<GlobalPositionMessage>(bytes), std::invalid_argument);
    auto packet = bytes;
    packet.push_back(std::byte{2});
    EXPECT_THROW(NetworkMessage{packet}, std::invalid_argument);
  }
}

TEST(ControlSerializationTest, EncodesAndDecodesEveryCommand) {
  using Command = ControllerStateMachine::CommandType;
  const std::array commands{Command::kUnguard, Command::kToManual,
                            Command::kToAutomatic, Command::kHold, Command::kLand};
  for (std::size_t index = 0; index < commands.size(); ++index) {
    const auto bytes = WithTimestamp(Bytes({static_cast<unsigned char>(index)}));
    EXPECT_EQ(Serialize(CommandMessage{simulation::TimePoint{42s}, commands[index]}),
              bytes);
    const auto decoded = Deserialize<CommandMessage>(bytes);
    EXPECT_EQ(decoded.command, commands[index]);
    EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
  }
}

TEST(ControlSerializationTest, RejectsUnknownCommands) {
  using Command = ControllerStateMachine::CommandType;
  for (const auto value : {5, 255}) {
    const CommandMessage message{{}, static_cast<Command>(value)};
    EXPECT_THROW(Serialize(message), std::invalid_argument);
    EXPECT_THROW(NetworkMessage{message}, std::invalid_argument);
    EXPECT_THROW(Deserialize<CommandMessage>(WithTimestamp(Bytes({static_cast<unsigned char>(value)}))),
                 std::invalid_argument);
  }
}

template <typename Message>
void ExpectControlPayload() {
  const Message message{simulation::TimePoint{42s},
                        {1.0, -2.0, 3.5}, {0.0, -0.0, 0.5}};
  EXPECT_EQ(Serialize(message), WithTimestamp(kSixDoubles));
  const auto decoded = Deserialize<Message>(WithTimestamp(kSixDoubles));
  EXPECT_EQ(decoded.timestamp, simulation::TimePoint{42s});
  EXPECT_DOUBLE_EQ(decoded.position.x_meters, 1.0);
  EXPECT_DOUBLE_EQ(decoded.position.y_meters, -2.0);
  EXPECT_DOUBLE_EQ(decoded.position.z_meters, 3.5);
  EXPECT_DOUBLE_EQ(decoded.orientation.roll_radians, 0.0);
  EXPECT_FALSE(std::signbit(decoded.orientation.roll_radians));
  EXPECT_DOUBLE_EQ(decoded.orientation.pitch_radians, -0.0);
  EXPECT_TRUE(std::signbit(decoded.orientation.pitch_radians));
  EXPECT_DOUBLE_EQ(decoded.orientation.yaw_radians, 0.5);
  EXPECT_EQ(Serialize(decoded), WithTimestamp(kSixDoubles));
}

TEST(ControlSerializationTest, ControlsEncodePositionThenOrientation) {
  ExpectControlPayload<ControlMessage>();
}

TEST(ControlSerializationTest, RejectsNonFiniteValuesInEveryControlField) {
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
    for (std::size_t field = 0; field < 6; ++field) {
      SCOPED_TRACE(field);
      ControlMessage message{simulation::TimePoint{42s}, {}, {}};
      const std::array fields{&message.position.x_meters, &message.position.y_meters,
                              &message.position.z_meters,
                              &message.orientation.roll_radians,
                              &message.orientation.pitch_radians,
                              &message.orientation.yaw_radians};
      *fields[field] = value;
      auto bytes = Serialize(message);
      EXPECT_THROW(Deserialize<ControlMessage>(bytes), std::invalid_argument);
      bytes.push_back(static_cast<std::byte>(MessageType::kControl));
      EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);
    }
  }
}

template <typename Message>
void ExpectNetworkMessage(const Message& message) {
  const NetworkMessage network{message};
  EXPECT_EQ(network.timestamp, message.timestamp);
  ASSERT_FALSE(network.data.empty());
  const std::vector<std::byte> payload(network.data.begin(), network.data.end() - 1);
  EXPECT_EQ(payload, Serialize(message));
  EXPECT_EQ(Deserialize<Message>(payload).timestamp, message.timestamp);
  EXPECT_EQ(network.data.back(), static_cast<std::byte>(network.type));
  auto later = message;
  later.timestamp += 1s;
  EXPECT_NE(NetworkMessage{later}.data, network.data);
}

TEST(TelemetrySerializationTest, NetworkMessageSerializesTimestampAndAppendsType) {
  const simulation::TimePoint time{123s + 456ms};
  ExpectNetworkMessage(HeartbeatMessage{time, State::kReady, true});
  ExpectNetworkMessage(AttitudeMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  ExpectNetworkMessage(GlobalPositionMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
  ExpectNetworkMessage(BatteryStatusMessage{time, 12.0, 10.0, 2.0});
  ExpectNetworkMessage(CommandMessage{time, ControllerStateMachine::CommandType::kHold});
  ExpectNetworkMessage(ControlMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}});
}

template <typename Message>
void ExpectTimestampRoundTrip(Message message) {
  for (const auto time : {simulation::TimePoint{}, simulation::TimePoint{1ns},
                          simulation::TimePoint{-1ns},
                          simulation::TimePoint{1ms}, simulation::TimePoint{-1ms},
                          simulation::TimePoint{-123s - 456ms - 789ns},
                          simulation::TimePoint{123s + 456ms + 789ns},
                          simulation::TimePoint::min(),
                          simulation::TimePoint::max()}) {
    message.timestamp = time;
    const auto bytes = Serialize(message);
    const simulation::TimePoint expected{
        std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch())};
    EXPECT_EQ(Deserialize<Message>(bytes).timestamp, expected);
    EXPECT_EQ(Serialize(Deserialize<Message>(bytes)), bytes);
  }
}

TEST(MessageSerializationTest, TimestampsRoundTripToMillisecondsForEveryMessageType) {
  ExpectTimestampRoundTrip(HeartbeatMessage{{}, State::kReady, true});
  ExpectTimestampRoundTrip(AttitudeMessage{{}, {}, {}});
  ExpectTimestampRoundTrip(GlobalPositionMessage{{}, {}, {}});
  ExpectTimestampRoundTrip(BatteryStatusMessage{{}, 12.0, 1.0, 2.0});
  ExpectTimestampRoundTrip(CommandMessage{{}, ControllerStateMachine::CommandType::kHold});
  ExpectTimestampRoundTrip(ControlMessage{{}, {}, {}});
}

TEST(MessageSerializationTest, TimestampUsesSignedLittleEndianMilliseconds) {
  const CommandMessage message{simulation::TimePoint{0x0102030405ms},
                               ControllerStateMachine::CommandType::kHold};
  const auto expected = Bytes({3, 5, 4, 3, 2, 1, 0, 0, 0});
  EXPECT_EQ(Serialize(message), expected);
  EXPECT_EQ(Deserialize<CommandMessage>(expected).timestamp, message.timestamp);
  const auto negative = Bytes({3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff});
  EXPECT_EQ(Serialize(CommandMessage{simulation::TimePoint{-1ms}, message.command}),
            negative);
  EXPECT_EQ(Deserialize<CommandMessage>(negative).timestamp, simulation::TimePoint{-1ms});
}

TEST(MessageSerializationTest, RejectsMillisecondsOutsideSimulationClockRange) {
  EXPECT_THROW(Deserialize<CommandMessage>(
      Bytes({3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f})),
      std::invalid_argument);
  EXPECT_THROW(Deserialize<CommandMessage>(
      Bytes({3, 0, 0, 0, 0, 0, 0, 0, 0x80})), std::invalid_argument);
}

template <typename Message>
void ExpectInvalidSizes(const Message& message) {
  auto bytes = Serialize(message);
  for (std::size_t size = 0; size < bytes.size(); ++size) {
    const std::vector<std::byte> truncated(bytes.begin(), bytes.begin() + size);
    EXPECT_THROW(Deserialize<Message>(truncated), std::invalid_argument);
  }
  bytes.push_back(std::byte{0});
  EXPECT_THROW(Deserialize<Message>(bytes), std::invalid_argument);
}

TEST(TelemetryDeserializationTest, RejectsTruncatedAndOversizedMessages) {
  ExpectInvalidSizes(HeartbeatMessage{{}, State::kReady, true});
  ExpectInvalidSizes(AttitudeMessage{{}, {}, {}});
  ExpectInvalidSizes(GlobalPositionMessage{{}, {}, {}});
  ExpectInvalidSizes(BatteryStatusMessage{{}, 12.0, 1.0, 2.0});
  ExpectInvalidSizes(CommandMessage{{}, ControllerStateMachine::CommandType::kHold});
  ExpectInvalidSizes(ControlMessage{{}, {}, {}});
}

TEST(TelemetryDeserializationTest, PreservesSpecialDoubleBitPatterns) {
  const auto bytes = WithTimestamp(Bytes({
      0, 0, 0, 0, 0, 0, 0xf0, 0x7f,
      0, 0, 0, 0, 0, 0, 0xf0, 0xff,
      0x42, 0, 0, 0, 0, 0, 0xf8, 0x7f}));
  const auto decoded = Deserialize<BatteryStatusMessage>(bytes);
  EXPECT_TRUE(std::isinf(decoded.voltage_volts));
  EXPECT_FALSE(std::signbit(decoded.voltage_volts));
  EXPECT_TRUE(std::isinf(decoded.current_amps));
  EXPECT_TRUE(std::signbit(decoded.current_amps));
  EXPECT_TRUE(std::isnan(decoded.charge_amp_hours));
  EXPECT_EQ(Serialize(decoded), bytes);
}

}
