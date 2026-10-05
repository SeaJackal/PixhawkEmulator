#pragma once

#include "message_type.h"
#include "simulation_time.h"
#include "telemetry_serialization.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

struct NetworkMessage {
  explicit NetworkMessage(std::vector<std::byte> bytes)
      : data(std::move(bytes)) {
    if (data.empty()) {
      throw std::invalid_argument("Missing message type");
    }
    type = static_cast<MessageType>(std::to_integer<unsigned char>(data.back()));
    const std::vector<std::byte> payload(data.begin(), data.end() - 1);
    switch (type) {
      case MessageType::kHeartbeat:
        timestamp = Deserialize<HeartbeatMessage>(payload).timestamp;
        break;
      case MessageType::kAttitude:
        timestamp = Deserialize<AttitudeMessage>(payload).timestamp;
        break;
      case MessageType::kGlobalPosition:
        timestamp = Deserialize<GlobalPositionMessage>(payload).timestamp;
        break;
      case MessageType::kBatteryStatus:
        timestamp = Deserialize<BatteryStatusMessage>(payload).timestamp;
        break;
      case MessageType::kCommand:
        timestamp = Deserialize<CommandMessage>(payload).timestamp;
        break;
      case MessageType::kAutomaticControl:
        timestamp = Deserialize<AutomaticControlMessage>(payload).timestamp;
        break;
      case MessageType::kManualControl:
        timestamp = Deserialize<ManualControlMessage>(payload).timestamp;
        break;
      default:
        throw std::invalid_argument("Unknown message type");
    }
  }

  template <typename Message>
  static constexpr MessageType GetType() = delete;

  template <typename Message>
  NetworkMessage(const Message& message)
      : timestamp(message.timestamp), type(GetType<Message>()),
        data(Serialize(message)) {
    data.push_back(static_cast<std::byte>(type));
  }

  simulation::TimePoint timestamp;
  MessageType type;
  std::vector<std::byte> data;
};

template <>
constexpr MessageType NetworkMessage::GetType<HeartbeatMessage>() {
  return MessageType::kHeartbeat;
}

template <>
constexpr MessageType NetworkMessage::GetType<AttitudeMessage>() {
  return MessageType::kAttitude;
}

template <>
constexpr MessageType NetworkMessage::GetType<GlobalPositionMessage>() {
  return MessageType::kGlobalPosition;
}

template <>
constexpr MessageType NetworkMessage::GetType<BatteryStatusMessage>() {
  return MessageType::kBatteryStatus;
}

template <>
constexpr MessageType NetworkMessage::GetType<CommandMessage>() {
  return MessageType::kCommand;
}

template <>
constexpr MessageType NetworkMessage::GetType<AutomaticControlMessage>() {
  return MessageType::kAutomaticControl;
}

template <>
constexpr MessageType NetworkMessage::GetType<ManualControlMessage>() {
  return MessageType::kManualControl;
}
