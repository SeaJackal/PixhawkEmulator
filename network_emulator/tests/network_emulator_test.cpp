#include "network_emulator.h"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>
#include <utility>

namespace {

using namespace std::chrono_literals;
using TimePoint = simulation::TimePoint;
using Type = MessageType;

constexpr std::array kTypes{Type::kHeartbeat, Type::kAttitude,
                            Type::kGlobalPosition, Type::kBatteryStatus,
                            Type::kCommand, Type::kControl};

NetworkMessage MessageAt(TimePoint timestamp, Type type = Type::kHeartbeat) {
  switch (type) {
    case Type::kHeartbeat:
      return NetworkMessage{HeartbeatMessage{
          timestamp, ControllerStateMachine::State::kReady, true}};
    case Type::kAttitude:
      return NetworkMessage{AttitudeMessage{timestamp, {}, {}}};
    case Type::kGlobalPosition:
      return NetworkMessage{GlobalPositionMessage{timestamp, {}, {}}};
    case Type::kBatteryStatus:
      return NetworkMessage{BatteryStatusMessage{timestamp, 12.0, 1.0, 2.0}};
    case Type::kCommand:
      return NetworkMessage{CommandMessage{
          timestamp, ControllerStateMachine::CommandType::kHold}};
    case Type::kControl:
      return NetworkMessage{ControlMessage{timestamp, {}, {}}};
  }
  throw std::invalid_argument("Unknown message type");
}

TEST(NetworkOutputEmulatorTest, AppliesLatencyAndPreservesMessageForEveryType) {
  const NetworkEmulator emulator{125ms};
  for (const auto type : kTypes) {
    const auto message = MessageAt(TimePoint{42s}, type);
    const auto result = emulator.Process(message);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->arrival_time, message.timestamp + 125ms);
    EXPECT_EQ(result->message.timestamp, message.timestamp);
    EXPECT_EQ(result->message.type, type);
    EXPECT_EQ(result->message.data, message.data);
  }
}

TEST(NetworkOutputEmulatorTest, AcceptsMovedMessageAndZeroLatency) {
  const NetworkEmulator emulator{0ms};
  auto message = MessageAt(TimePoint{42s});
  const auto original = message;
  const auto result = emulator.Process(std::move(message));
  ASSERT_TRUE(result);
  EXPECT_EQ(result->arrival_time, original.timestamp);
  EXPECT_EQ(result->message.timestamp, original.timestamp);
  EXPECT_EQ(result->message.data, original.data);
}

TEST(NetworkOutputEmulatorTest, DelayMatchesOriginalTimestampAndExactType) {
  for (const auto filtered_type : kTypes) {
    NetworkEmulator emulator{2s};
    emulator.AddDelayFilter(TimePoint{10s}, TimePoint{11s}, filtered_type, 3s);
    for (const auto type : kTypes) {
      const auto message = MessageAt(TimePoint{10500ms}, type);
      const auto result = emulator.Process(message);
      ASSERT_TRUE(result);
      EXPECT_EQ(result->arrival_time,
                message.timestamp + (type == filtered_type ? 5s : 2s));
      EXPECT_EQ(result->message.timestamp, message.timestamp);
      EXPECT_EQ(result->message.data, message.data);
    }
  }
}

TEST(NetworkOutputEmulatorTest, DropMatchesOnlySpecifiedType) {
  for (const auto filtered_type : kTypes) {
    NetworkEmulator emulator{2s};
    emulator.AddDropFilter(TimePoint{10s}, TimePoint{11s}, filtered_type);
    for (const auto type : kTypes) {
      const auto result = emulator.Process(MessageAt(TimePoint{10500ms}, type));
      EXPECT_EQ(result.has_value(), type != filtered_type);
    }
  }
}

TEST(NetworkOutputEmulatorTest, DelayIncludesStartAndExcludesEnd) {
  NetworkEmulator emulator{10ms};
  emulator.AddDelayFilter(TimePoint{1s}, TimePoint{2s}, Type::kHeartbeat, 100ms);
  for (const auto time : {999ms, 1000ms, 1999ms, 2000ms, 2001ms}) {
    const auto result = emulator.Process(MessageAt(TimePoint{time}));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->arrival_time,
              TimePoint{time} + 10ms + (time >= 1s && time < 2s ? 100ms : 0ms));
  }
}

TEST(NetworkOutputEmulatorTest, DropIncludesStartAndExcludesEnd) {
  NetworkEmulator emulator{10ms};
  emulator.AddDropFilter(TimePoint{1s}, TimePoint{2s}, Type::kHeartbeat);
  for (const auto time : {999ms, 1000ms, 1999ms, 2000ms, 2001ms}) {
    const auto result = emulator.Process(MessageAt(TimePoint{time}));
    EXPECT_EQ(result.has_value(), time < 1s || time >= 2s);
  }
}

TEST(NetworkOutputEmulatorTest, EmptyIntervalsNeverMatch) {
  NetworkEmulator emulator{10ms};
  emulator.AddDropFilter(TimePoint{1s}, TimePoint{1s}, Type::kHeartbeat);
  emulator.AddDelayFilter(TimePoint{1s}, TimePoint{1s}, Type::kHeartbeat, 100ms);
  for (const auto time : {999ms, 1000ms, 1001ms}) {
    const auto result = emulator.Process(MessageAt(TimePoint{time}));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->arrival_time, TimePoint{time} + 10ms);
  }
}

TEST(NetworkOutputEmulatorTest, FirstMatchingDelayWinsWithoutSummingOrTakingMaximum) {
  NetworkEmulator emulator{10ms};
  emulator.AddDelayFilter(TimePoint{0s}, TimePoint{1s}, Type::kHeartbeat, 1s);
  emulator.AddDelayFilter(TimePoint{0s}, TimePoint{3s}, Type::kAttitude, 1s);
  emulator.AddDelayFilter(TimePoint{1s}, TimePoint{3s}, Type::kHeartbeat, 100ms);
  emulator.AddDelayFilter(TimePoint{0s}, TimePoint{4s}, Type::kHeartbeat, 500ms);
  const auto result = emulator.Process(MessageAt(TimePoint{2s}));
  ASSERT_TRUE(result);
  EXPECT_EQ(result->arrival_time, TimePoint{2110ms});
}

TEST(NetworkOutputEmulatorTest, EarlierDelayOverridesLaterDropEvenWhenDelayIsZero) {
  for (const auto delay : {0ms, 100ms}) {
    NetworkEmulator emulator{10ms};
    emulator.AddDelayFilter(TimePoint{1s}, TimePoint{3s}, Type::kHeartbeat, delay);
    emulator.AddDropFilter(TimePoint{1s}, TimePoint{3s}, Type::kHeartbeat);
    const auto result = emulator.Process(MessageAt(TimePoint{2s}));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->arrival_time, TimePoint{2s} + 10ms + delay);
  }
}

TEST(NetworkOutputEmulatorTest, EarlierDropOverridesLaterDelay) {
  NetworkEmulator emulator{10ms};
  emulator.AddDropFilter(TimePoint{1s}, TimePoint{3s}, Type::kHeartbeat);
  emulator.AddDelayFilter(TimePoint{1s}, TimePoint{3s}, Type::kHeartbeat, 100ms);
  EXPECT_FALSE(emulator.Process(MessageAt(TimePoint{2s})));
}

TEST(NetworkOutputEmulatorTest, FiltersPersistAcrossRepeatedAndOutOfOrderCalls) {
  NetworkEmulator emulator{10ms};
  emulator.AddDelayFilter(TimePoint{1s}, TimePoint{2s}, Type::kHeartbeat, 100ms);
  emulator.AddDropFilter(TimePoint{2s}, TimePoint{3s}, Type::kHeartbeat);
  for (const auto time : {3s, 2s, 1s, 0s, 1s, 2s, 3s}) {
    const auto result = emulator.Process(MessageAt(TimePoint{time}));
    if (time == 2s) {
      EXPECT_FALSE(result);
    } else {
      ASSERT_TRUE(result);
      EXPECT_EQ(result->arrival_time,
                TimePoint{time} + 10ms + (time == 1s ? 100ms : 0ms));
    }
  }
}

TEST(NetworkOutputEmulatorTest, ChecksLatencyAtTimestampLimits) {
  const NetworkEmulator emulator{10ms};
  const auto boundary = TimePoint::max() - 10ms;
  const auto result = emulator.Process(MessageAt(boundary));
  ASSERT_TRUE(result);
  EXPECT_EQ(result->arrival_time, TimePoint::max());
  EXPECT_THROW(emulator.Process(MessageAt(boundary + 1ns)),
               std::invalid_argument);
  const auto oldest = emulator.Process(MessageAt(TimePoint::min()));
  ASSERT_TRUE(oldest);
  EXPECT_EQ(oldest->arrival_time, TimePoint::min() + 10ms);
  const auto latest = NetworkEmulator{0ms}.Process(MessageAt(TimePoint::max()));
  ASSERT_TRUE(latest);
  EXPECT_EQ(latest->arrival_time, TimePoint::max());
}

TEST(NetworkOutputEmulatorTest, ChecksFilterDelayAfterLatency) {
  NetworkEmulator emulator{1ms};
  const auto boundary = TimePoint::max() - 11ms;
  emulator.AddDelayFilter(boundary, TimePoint::max(), Type::kHeartbeat, 10ms);
  const auto result = emulator.Process(MessageAt(boundary));
  ASSERT_TRUE(result);
  EXPECT_EQ(result->arrival_time, TimePoint::max());
  EXPECT_THROW(emulator.Process(MessageAt(boundary + 1ns)),
               std::invalid_argument);
}

TEST(NetworkOutputEmulatorTest, RejectsNegativeLatency) {
  EXPECT_DEBUG_DEATH(NetworkEmulator{-1ms}, "");
}

TEST(NetworkOutputEmulatorTest, RejectsNegativeDelay) {
  NetworkEmulator emulator{0ms};
  EXPECT_DEBUG_DEATH(
      emulator.AddDelayFilter(TimePoint{1s}, TimePoint{2s}, Type::kHeartbeat, -1ms),
      "");
}

TEST(NetworkOutputEmulatorTest, RejectsReversedIntervalsForBothKinds) {
  NetworkEmulator emulator{0ms};
  EXPECT_DEBUG_DEATH(
      emulator.AddDelayFilter(TimePoint{2s}, TimePoint{1s}, Type::kHeartbeat, 0ms),
      "");
  EXPECT_DEBUG_DEATH(
      emulator.AddDropFilter(TimePoint{2s}, TimePoint{1s}, Type::kHeartbeat), "");
}

}
