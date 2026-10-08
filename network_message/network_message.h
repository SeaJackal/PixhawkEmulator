#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "message_type.h"
#include "simulation_time.h"
#include "telemetry_serialization.h"

struct NetworkMessage {
  explicit NetworkMessage(std::vector<std::byte> bytes);

  template <typename Message>
  static constexpr MessageType GetType() = delete;

  template <typename Message>
  NetworkMessage(const Message& message);

  simulation::TimePoint timestamp;
  MessageType type;
  std::vector<std::byte> data;
};

template <>
constexpr MessageType NetworkMessage::GetType<HeartbeatMessage>();

template <>
constexpr MessageType NetworkMessage::GetType<AttitudeMessage>();

template <>
constexpr MessageType NetworkMessage::GetType<GlobalPositionMessage>();

template <>
constexpr MessageType NetworkMessage::GetType<BatteryStatusMessage>();

template <>
constexpr MessageType NetworkMessage::GetType<CommandMessage>();

template <>
constexpr MessageType NetworkMessage::GetType<ControlMessage>();

// -----------------------------------------------------------------------------

inline NetworkMessage::NetworkMessage(std::vector<std::byte> bytes)
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
    case MessageType::kControl:
      timestamp = Deserialize<ControlMessage>(payload).timestamp;
      break;
    default:
      throw std::invalid_argument("Unknown message type");
  }
}

template <typename Message>
NetworkMessage::NetworkMessage(const Message& message)
    : timestamp(message.timestamp),
      type(GetType<Message>()),
      data(Serialize(message)) {
  data.push_back(static_cast<std::byte>(type));
}

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
constexpr MessageType NetworkMessage::GetType<ControlMessage>() {
  return MessageType::kControl;
}
