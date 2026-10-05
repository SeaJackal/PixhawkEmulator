#pragma once

#include "control_messages.h"
#include "telemetry.h"

#include <cstddef>
#include <vector>

std::vector<std::byte> Serialize(const HeartbeatMessage& message);
std::vector<std::byte> Serialize(const AttitudeMessage& message);
std::vector<std::byte> Serialize(const GlobalPositionMessage& message);
std::vector<std::byte> Serialize(const BatteryStatusMessage& message);
std::vector<std::byte> Serialize(const Telemetry& telemetry);
std::vector<std::byte> Serialize(const CommandMessage& message);
std::vector<std::byte> Serialize(const AutomaticControlMessage& message);
std::vector<std::byte> Serialize(const ManualControlMessage& message);

template <typename Message>
Message Deserialize(const std::vector<std::byte>& bytes) = delete;

template <>
HeartbeatMessage Deserialize<HeartbeatMessage>(const std::vector<std::byte>& bytes);
template <>
AttitudeMessage Deserialize<AttitudeMessage>(const std::vector<std::byte>& bytes);
template <>
GlobalPositionMessage Deserialize<GlobalPositionMessage>(
    const std::vector<std::byte>& bytes);
template <>
BatteryStatusMessage Deserialize<BatteryStatusMessage>(
    const std::vector<std::byte>& bytes);
template <>
CommandMessage Deserialize<CommandMessage>(const std::vector<std::byte>& bytes);
template <>
AutomaticControlMessage Deserialize<AutomaticControlMessage>(
    const std::vector<std::byte>& bytes);
template <>
ManualControlMessage Deserialize<ManualControlMessage>(
    const std::vector<std::byte>& bytes);
