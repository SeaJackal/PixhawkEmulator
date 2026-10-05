#include "network_message.h"

#include <gtest/gtest.h>

#include <stdexcept>
#include <type_traits>
#include <utility>

namespace {

using namespace std::chrono_literals;

template <typename Message>
void ExpectPacket(const Message& message, MessageType type, std::byte code) {
  static_assert(std::is_convertible_v<Message, NetworkMessage>);
  constexpr auto inferred_type = NetworkMessage::GetType<Message>();
  EXPECT_EQ(inferred_type, type);
  const NetworkMessage network = message;
  auto expected = Serialize(message);
  expected.push_back(code);
  EXPECT_EQ(network.timestamp, message.timestamp);
  EXPECT_EQ(network.type, type);
  EXPECT_EQ(network.data, expected);
  const NetworkMessage decoded{expected};
  EXPECT_EQ(decoded.timestamp, message.timestamp);
  EXPECT_EQ(decoded.type, type);
  EXPECT_EQ(decoded.data, expected);
  for (std::size_t size = 0; size < expected.size() - 1; ++size) {
    std::vector<std::byte> truncated(expected.begin(), expected.begin() + size);
    truncated.push_back(code);
    EXPECT_THROW(NetworkMessage{truncated}, std::invalid_argument);
  }
  expected.insert(expected.end() - 1, std::byte{0});
  EXPECT_THROW(NetworkMessage{expected}, std::invalid_argument);
}

TEST(NetworkMessageTest, ConstructorsRoundTripEveryMessageType) {
  const simulation::TimePoint time{42s + 125ms};
  ExpectPacket(HeartbeatMessage{time, ControllerStateMachine::State::kReady, true},
               MessageType::kHeartbeat, std::byte{0});
  ExpectPacket(AttitudeMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}},
               MessageType::kAttitude, std::byte{1});
  ExpectPacket(GlobalPositionMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}},
               MessageType::kGlobalPosition, std::byte{2});
  ExpectPacket(BatteryStatusMessage{time, 12.0, 10.0, 2.0},
               MessageType::kBatteryStatus, std::byte{3});
  ExpectPacket(CommandMessage{time, ControllerStateMachine::CommandType::kHold},
               MessageType::kCommand, std::byte{4});
  ExpectPacket(AutomaticControlMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}},
               MessageType::kAutomaticControl, std::byte{5});
  ExpectPacket(ManualControlMessage{time, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}},
               MessageType::kManualControl, std::byte{6});
}

TEST(NetworkMessageTest, OwnsSerializedBytesAfterTemporaryArgumentExpires) {
  const NetworkMessage network{
      HeartbeatMessage{simulation::TimePoint{7s},
                       ControllerStateMachine::State::kReady, true}};

  EXPECT_EQ(network.timestamp, simulation::TimePoint{7s});
  EXPECT_EQ(network.type, MessageType::kHeartbeat);
  EXPECT_EQ(network.data,
            (std::vector<std::byte>{std::byte{2}, std::byte{1},
                                    std::byte{0x58}, std::byte{0x1b},
                                    std::byte{0x00}, std::byte{0x00},
                                    std::byte{0x00}, std::byte{0x00},
                                    std::byte{0x00}, std::byte{0x00},
                                    std::byte{0}}));
}

TEST(NetworkMessageTest, ByteConstructorAcceptsMovedBytesAndRestoresMilliseconds) {
  const NetworkMessage original{CommandMessage{
      simulation::TimePoint{-2s - 123ms - 456ns},
      ControllerStateMachine::CommandType::kHold}};
  auto bytes = original.data;
  const NetworkMessage decoded{std::move(bytes)};
  EXPECT_EQ(decoded.timestamp, simulation::TimePoint{-2s - 123ms});
  EXPECT_EQ(decoded.type, original.type);
  EXPECT_EQ(decoded.data, original.data);
}

TEST(NetworkMessageTest, ByteConstructorRejectsMissingOrUnknownType) {
  EXPECT_THROW(NetworkMessage{std::vector<std::byte>{}}, std::invalid_argument);
  for (unsigned int code = 7; code <= 255; ++code) {
    const std::vector<std::byte> bytes{static_cast<std::byte>(code)};
    EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);
  }
}

TEST(NetworkMessageTest, ByteConstructorValidatesPayloadContents) {
  const NetworkMessage heartbeat{HeartbeatMessage{
      {}, ControllerStateMachine::State::kReady, true}};
  auto bytes = heartbeat.data;
  bytes[0] = std::byte{255};
  EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);
  bytes = heartbeat.data;
  bytes[1] = std::byte{2};
  EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);

  const NetworkMessage command{CommandMessage{
      {}, ControllerStateMachine::CommandType::kHold}};
  bytes = command.data;
  bytes[0] = std::byte{255};
  EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);
  bytes = command.data;
  for (std::size_t index = 1; index < bytes.size() - 1; ++index) {
    bytes[index] = std::byte{0x7f};
  }
  EXPECT_THROW(NetworkMessage{bytes}, std::invalid_argument);
}

TEST(NetworkMessageTest, PropagatesSerializationFailure) {
  const HeartbeatMessage message{
      simulation::TimePoint{1s}, static_cast<ControllerStateMachine::State>(99), false};

  EXPECT_THROW(NetworkMessage{message}, std::invalid_argument);
}

TEST(NetworkMessageTest, CopyAndMovePreserveTimestampTypeAndBytes) {
  const BatteryStatusMessage message{simulation::TimePoint{2s}, 12.0, 10.0, 2.0};
  const NetworkMessage original{message};

  NetworkMessage copied{original};
  EXPECT_EQ(copied.timestamp, original.timestamp);
  EXPECT_EQ(copied.type, original.type);
  EXPECT_EQ(copied.data, original.data);

  const NetworkMessage moved{std::move(copied)};
  EXPECT_EQ(moved.timestamp, original.timestamp);
  EXPECT_EQ(moved.type, original.type);
  EXPECT_EQ(moved.data, original.data);
}

}
