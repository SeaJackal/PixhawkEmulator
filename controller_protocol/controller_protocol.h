#pragma once

#include "network_message.h"

#include <cmath>
#include <stdexcept>
#include <variant>

class ControllerProtocol {
 public:
  using Command = std::variant<CommandMessage, ManualControlMessage,
                               AutomaticControlMessage>;

  static std::vector<NetworkMessage> Encode(const Telemetry& telemetry) {
    std::vector<NetworkMessage> messages;
    if (telemetry.heartbeat) messages.emplace_back(*telemetry.heartbeat);
    if (telemetry.attitude) messages.emplace_back(*telemetry.attitude);
    if (telemetry.global_position) messages.emplace_back(*telemetry.global_position);
    if (telemetry.battery_status) messages.emplace_back(*telemetry.battery_status);
    return messages;
  }

  static Command Decode(const NetworkMessage& message) {
    if (message.data.empty()) {
      throw std::invalid_argument("Empty command");
    }
    const std::vector<std::byte> payload(message.data.begin(), message.data.end() - 1);
    switch (message.type) {
      case MessageType::kCommand:
        return Deserialize<CommandMessage>(payload);
      case MessageType::kManualControl: {
        auto command = Deserialize<ManualControlMessage>(payload);
        Validate(command.position, command.orientation);
        return command;
      }
      case MessageType::kAutomaticControl: {
        auto command = Deserialize<AutomaticControlMessage>(payload);
        Validate(command.position, command.orientation);
        return command;
      }
      default:
        throw std::invalid_argument("Expected controller command");
    }
  }

  template <typename ControllerType>
  static bool Deliver(ControllerType& controller, const NetworkMessage& message) {
    return std::visit([&controller](const auto& command) {
      return Apply(controller, command);
    }, Decode(message));
  }

 private:
  static void Validate(const Position& position, const Orientation& orientation) {
    if (!std::isfinite(position.x_meters) || !std::isfinite(position.y_meters) ||
        !std::isfinite(position.z_meters) || !std::isfinite(orientation.roll_radians) ||
        !std::isfinite(orientation.pitch_radians) || !std::isfinite(orientation.yaw_radians)) {
      throw std::invalid_argument("Non-finite control target");
    }
  }

  template <typename ControllerType>
  static bool Apply(ControllerType& controller, const CommandMessage& command) {
    return controller.Command(command.command);
  }

  template <typename ControllerType>
  static bool Apply(ControllerType& controller, const ManualControlMessage& command) {
    return controller.SetManualControl(command.position, command.orientation);
  }

  template <typename ControllerType>
  static bool Apply(ControllerType& controller, const AutomaticControlMessage& command) {
    return controller.SetAutomaticControl(command.position, command.orientation);
  }
};
