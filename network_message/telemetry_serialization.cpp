#include "telemetry_serialization.h"

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace {

constexpr std::size_t kDoubleBytes = 8;
constexpr std::size_t kTimestampBytes = 8;
constexpr std::size_t kMessageTypeBytes = 1;
constexpr std::size_t kPresenceMaskBytes = 1;
constexpr std::size_t kHeartbeatFieldsBytes = 2;
constexpr std::size_t kCommandFieldsBytes = 1;
constexpr std::size_t kSixDoubleFieldsBytes = 6 * kDoubleBytes;
constexpr std::size_t kBatteryFieldsBytes = 3 * kDoubleBytes;
constexpr std::size_t kHeartbeatBytes = kHeartbeatFieldsBytes + kTimestampBytes;
constexpr std::size_t kCommandBytes = kCommandFieldsBytes + kTimestampBytes;
constexpr std::size_t kAttitudeBytes = kSixDoubleFieldsBytes + kTimestampBytes;
constexpr std::size_t kGlobalPositionBytes = kSixDoubleFieldsBytes + 1 + kTimestampBytes;
constexpr std::size_t kControlBytes = kSixDoubleFieldsBytes + kTimestampBytes;
constexpr std::size_t kBatteryStatusBytes = kBatteryFieldsBytes + kTimestampBytes;
constexpr std::size_t kTelemetryBytes = kPresenceMaskBytes + kHeartbeatBytes +
    kAttitudeBytes + kGlobalPositionBytes + kBatteryStatusBytes;

static_assert(std::ratio_equal_v<simulation::Duration::period, std::nano>);
static_assert(std::numeric_limits<simulation::Duration::rep>::is_signed);
static_assert(std::numeric_limits<simulation::Duration::rep>::digits == 63);

void CheckSize(const std::vector<std::byte>& bytes, std::size_t expected) {
  if (bytes.size() != expected) {
    throw std::invalid_argument("Invalid message payload size");
  }
}

std::uint64_t ReadUint64(const std::vector<std::byte>& bytes, std::size_t offset) {
  std::uint64_t bits = 0;
  for (int shift = 0; shift < 64; shift += 8) {
    bits |= std::to_integer<std::uint64_t>(bytes[offset++]) << shift;
  }
  return bits;
}

double ReadDouble(const std::vector<std::byte>& bytes, std::size_t offset) {
  const auto bits = ReadUint64(bytes, offset);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

simulation::TimePoint ReadTimestamp(const std::vector<std::byte>& bytes) {
  const auto bits = ReadUint64(bytes, bytes.size() - kTimestampBytes);
  const auto count = bits <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
      ? static_cast<std::int64_t>(bits)
      : -1 - static_cast<std::int64_t>(std::numeric_limits<std::uint64_t>::max() - bits);
  const auto min_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      simulation::Duration::min()).count();
  const auto max_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      simulation::Duration::max()).count();
  if (count < min_milliseconds || count > max_milliseconds) {
    throw std::invalid_argument("Timestamp exceeds simulation clock range");
  }
  return simulation::TimePoint{
      std::chrono::duration_cast<simulation::Duration>(std::chrono::milliseconds{count})};
}

ControllerStateMachine::State DeserializeState(std::byte value) {
  using State = ControllerStateMachine::State;
  switch (std::to_integer<unsigned char>(value)) {
    case 0:
      return State::kGuarded;
    case 1:
      return State::kNotReady;
    case 2:
      return State::kReady;
    case 3:
      return State::kManual;
    case 4:
      return State::kAutomatic;
    case 5:
      return State::kHolding;
  }
  throw std::invalid_argument("Unknown heartbeat state");
}

template <typename Message>
Message ReadMessage(const std::vector<std::byte>& bytes, std::size_t& offset,
                    std::size_t size) {
  if (size > bytes.size() - offset) {
    throw std::invalid_argument("Truncated telemetry payload");
  }
  const auto message = Deserialize<Message>(
      std::vector<std::byte>(bytes.begin() + offset, bytes.begin() + offset + size));
  offset += size;
  return message;
}

void AppendUint64(std::vector<std::byte>& bytes, std::uint64_t bits) {
  for (int shift = 0; shift < 64; shift += 8) {
    bytes.push_back(static_cast<std::byte>((bits >> shift) & 0xff));
  }
}

void AppendTimestamp(std::vector<std::byte>& bytes, simulation::TimePoint timestamp) {
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      timestamp.time_since_epoch()).count();
  AppendUint64(bytes, static_cast<std::uint64_t>(milliseconds));
}

void AppendDouble(std::vector<std::byte>& bytes, double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t));
  static_assert(std::numeric_limits<double>::is_iec559);
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendUint64(bytes, bits);
}

std::vector<std::byte> SerializeControl(simulation::TimePoint timestamp,
                                      const Position& position,
                                      const Orientation& orientation) {
  std::vector<std::byte> bytes;
  bytes.reserve(kControlBytes + kMessageTypeBytes);
  AppendDouble(bytes, position.x_meters);
  AppendDouble(bytes, position.y_meters);
  AppendDouble(bytes, position.z_meters);
  AppendDouble(bytes, orientation.roll_radians);
  AppendDouble(bytes, orientation.pitch_radians);
  AppendDouble(bytes, orientation.yaw_radians);
  AppendTimestamp(bytes, timestamp);
  return bytes;
}

std::byte SerializeState(ControllerStateMachine::State state) {
  using State = ControllerStateMachine::State;
  switch (state) {
    case State::kGuarded:
      return std::byte{0};
    case State::kNotReady:
      return std::byte{1};
    case State::kReady:
      return std::byte{2};
    case State::kManual:
      return std::byte{3};
    case State::kAutomatic:
      return std::byte{4};
    case State::kHolding:
      return std::byte{5};
  }
  throw std::invalid_argument("Unknown heartbeat state");
}

template <typename Message>
void AppendOptional(std::vector<std::byte>& bytes,
                    const std::optional<Message>& message, std::byte flag) {
  if (message) {
    bytes.front() |= flag;
    const auto payload = Serialize(*message);
    bytes.insert(bytes.end(), payload.begin(), payload.end());
  }
}

}

std::vector<std::byte> Serialize(const HeartbeatMessage& message) {
  std::vector<std::byte> bytes;
  bytes.reserve(kHeartbeatBytes + kMessageTypeBytes);
  bytes.push_back(SerializeState(message.state));
  bytes.push_back(message.ready ? std::byte{1} : std::byte{0});
  AppendTimestamp(bytes, message.timestamp);
  return bytes;
}

std::vector<std::byte> Serialize(const CommandMessage& message) {
  std::vector<std::byte> bytes;
  bytes.reserve(kCommandBytes + kMessageTypeBytes);
  using Command = ControllerStateMachine::CommandType;
  if (message.command > Command::kLand) {
    throw std::invalid_argument("Unknown command");
  }
  bytes.push_back(static_cast<std::byte>(message.command));
  AppendTimestamp(bytes, message.timestamp);
  return bytes;
}

std::vector<std::byte> Serialize(const AutomaticControlMessage& message) {
  return SerializeControl(message.timestamp, message.position, message.orientation);
}

std::vector<std::byte> Serialize(const ManualControlMessage& message) {
  return SerializeControl(message.timestamp, message.position, message.orientation);
}

std::vector<std::byte> Serialize(const AttitudeMessage& message) {
  std::vector<std::byte> bytes;
  bytes.reserve(kAttitudeBytes + kMessageTypeBytes);
  AppendDouble(bytes, message.orientation.roll_radians);
  AppendDouble(bytes, message.orientation.pitch_radians);
  AppendDouble(bytes, message.orientation.yaw_radians);
  AppendDouble(bytes, message.angular_velocity.roll_radians_per_second);
  AppendDouble(bytes, message.angular_velocity.pitch_radians_per_second);
  AppendDouble(bytes, message.angular_velocity.yaw_radians_per_second);
  AppendTimestamp(bytes, message.timestamp);
  return bytes;
}

std::vector<std::byte> Serialize(const GlobalPositionMessage& message) {
  std::vector<std::byte> bytes;
  bytes.reserve(kGlobalPositionBytes + kMessageTypeBytes);
  AppendDouble(bytes, message.position.x_meters);
  AppendDouble(bytes, message.position.y_meters);
  AppendDouble(bytes, message.position.z_meters);
  AppendDouble(bytes, message.velocity.x_meters_per_second);
  AppendDouble(bytes, message.velocity.y_meters_per_second);
  AppendDouble(bytes, message.velocity.z_meters_per_second);
  bytes.push_back(message.gnss_valid ? std::byte{1} : std::byte{0});
  AppendTimestamp(bytes, message.timestamp);
  return bytes;
}

std::vector<std::byte> Serialize(const BatteryStatusMessage& message) {
  std::vector<std::byte> bytes;
  bytes.reserve(kBatteryStatusBytes + kMessageTypeBytes);
  AppendDouble(bytes, message.voltage_volts);
  AppendDouble(bytes, message.current_amps);
  AppendDouble(bytes, message.charge_amp_hours);
  AppendTimestamp(bytes, message.timestamp);
  return bytes;
}

std::vector<std::byte> Serialize(const Telemetry& telemetry) {
  std::vector<std::byte> bytes{std::byte{0}};
  bytes.reserve(kTelemetryBytes);
  AppendOptional(bytes, telemetry.heartbeat, std::byte{0x01});
  AppendOptional(bytes, telemetry.attitude, std::byte{0x02});
  AppendOptional(bytes, telemetry.global_position, std::byte{0x04});
  AppendOptional(bytes, telemetry.battery_status, std::byte{0x08});
  return bytes;
}

template <>
CommandMessage Deserialize<CommandMessage>(const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kCommandBytes);
  using Command = ControllerStateMachine::CommandType;
  const auto command = static_cast<Command>(std::to_integer<std::uint8_t>(bytes.front()));
  if (command > Command::kLand) {
    throw std::invalid_argument("Unknown command");
  }
  return {ReadTimestamp(bytes), command};
}

template <>
AutomaticControlMessage Deserialize<AutomaticControlMessage>(
    const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kControlBytes);
  return {ReadTimestamp(bytes),
          {ReadDouble(bytes, 0), ReadDouble(bytes, kDoubleBytes),
           ReadDouble(bytes, 2 * kDoubleBytes)},
          {ReadDouble(bytes, 3 * kDoubleBytes), ReadDouble(bytes, 4 * kDoubleBytes),
           ReadDouble(bytes, 5 * kDoubleBytes)}};
}

template <>
ManualControlMessage Deserialize<ManualControlMessage>(
    const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kControlBytes);
  return {ReadTimestamp(bytes),
          {ReadDouble(bytes, 0), ReadDouble(bytes, kDoubleBytes),
           ReadDouble(bytes, 2 * kDoubleBytes)},
          {ReadDouble(bytes, 3 * kDoubleBytes), ReadDouble(bytes, 4 * kDoubleBytes),
           ReadDouble(bytes, 5 * kDoubleBytes)}};
}

template <>
HeartbeatMessage Deserialize<HeartbeatMessage>(const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kHeartbeatBytes);
  if (bytes[1] != std::byte{0} && bytes[1] != std::byte{1}) {
    throw std::invalid_argument("Invalid heartbeat readiness flag");
  }
  return {ReadTimestamp(bytes), DeserializeState(bytes[0]), bytes[1] == std::byte{1}};
}

template <>
AttitudeMessage Deserialize<AttitudeMessage>(const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kAttitudeBytes);
  return {ReadTimestamp(bytes),
          {ReadDouble(bytes, 0), ReadDouble(bytes, kDoubleBytes),
           ReadDouble(bytes, 2 * kDoubleBytes)},
          {ReadDouble(bytes, 3 * kDoubleBytes), ReadDouble(bytes, 4 * kDoubleBytes),
           ReadDouble(bytes, 5 * kDoubleBytes)}};
}

template <>
GlobalPositionMessage Deserialize<GlobalPositionMessage>(
    const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kGlobalPositionBytes);
  if (bytes[kSixDoubleFieldsBytes] != std::byte{0} &&
      bytes[kSixDoubleFieldsBytes] != std::byte{1}) {
    throw std::invalid_argument("Invalid GNSS validity flag");
  }
  return {ReadTimestamp(bytes),
          {ReadDouble(bytes, 0), ReadDouble(bytes, kDoubleBytes),
           ReadDouble(bytes, 2 * kDoubleBytes)},
          {ReadDouble(bytes, 3 * kDoubleBytes), ReadDouble(bytes, 4 * kDoubleBytes),
           ReadDouble(bytes, 5 * kDoubleBytes)},
          bytes[kSixDoubleFieldsBytes] == std::byte{1}};
}

template <>
BatteryStatusMessage Deserialize<BatteryStatusMessage>(
    const std::vector<std::byte>& bytes) {
  CheckSize(bytes, kBatteryStatusBytes);
  return {ReadTimestamp(bytes), ReadDouble(bytes, 0),
          ReadDouble(bytes, kDoubleBytes), ReadDouble(bytes, 2 * kDoubleBytes)};
}
