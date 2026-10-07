#pragma once

#include "network_message.h"

#include <cmath>
#include <stdexcept>
#include <variant>

enum class ControlSource { kAutomatic, kManual };

class ControllerProtocol {
 public:
  using Command = std::variant<CommandMessage, ControlMessage>;

  static std::vector<NetworkMessage> Encode(const Telemetry& telemetry) {
    std::vector<NetworkMessage> messages;
    if (telemetry.heartbeat) messages.emplace_back(*telemetry.heartbeat);
    if (telemetry.attitude) messages.emplace_back(*telemetry.attitude);
    if (telemetry.global_position) messages.emplace_back(*telemetry.global_position);
    if (telemetry.battery_status) messages.emplace_back(*telemetry.battery_status);
    return messages;
  }

  static Command Decode(const NetworkMessage& message) {
    const std::vector<std::byte> payload(message.data.begin(), message.data.end() - 1);
    switch (message.type) {
      case MessageType::kCommand:
        return Deserialize<CommandMessage>(payload);
      case MessageType::kControl: {
        auto command = Deserialize<ControlMessage>(payload);
        Validate(command.position, command.orientation);
        return command;
      }
      default:
        throw std::invalid_argument("Expected controller command");
    }
  }

  template <typename ControllerType>
  static bool Deliver(ControllerType& controller, const NetworkMessage& message,
                      ControlSource source) {
    return std::visit([&controller, source](const auto& command) {
      return Apply(controller, command, source);
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
  static bool Apply(ControllerType& controller, const CommandMessage& command,
                    ControlSource) {
    return controller.Command(command.command);
  }

  template <typename ControllerType>
  static bool Apply(ControllerType& controller, const ControlMessage& command,
                    ControlSource source) {
    if (source == ControlSource::kManual) {
      return controller.SetManualControl(command.position, command.orientation);
    }
    return controller.SetAutomaticControl(command.position, command.orientation);
  }
};
