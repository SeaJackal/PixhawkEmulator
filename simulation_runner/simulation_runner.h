#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "control_messages.h"
#include "logger.h"
#include "message_type.h"
#include "network_emulator.h"
#include "network_message.h"
#include "simulation_time.h"
#include "telemetry.h"
#include "telemetry_serialization.h"

template <typename ControllerType, typename Transport>
class SimulationRunner {
 public:
  using TimePoint = simulation::TimePoint;
  using RecordSink = std::function<void(const Logger::Record&)>;

  SimulationRunner(ControllerType& controller, Transport& transport,
                   Transport& manual_transport, NetworkEmulator inbound,
                   NetworkEmulator outbound);

  SimulationRunner(ControllerType& controller, Transport& transport,
                   Transport& manual_transport, NetworkEmulator inbound,
                   NetworkEmulator outbound, Logger& logger,
                   RecordSink record_sink = {});

  ~SimulationRunner() = default;
  SimulationRunner(const SimulationRunner&) = delete;
  SimulationRunner& operator=(const SimulationRunner&) = delete;
  SimulationRunner(SimulationRunner&&) = delete;
  SimulationRunner& operator=(SimulationRunner&&) = delete;

  void RunOnce();

  template <typename StopRequested>
  void Run(StopRequested stop_requested);

 private:
  enum class ControlSource { kAutomatic, kManual };

  static std::vector<NetworkMessage> Encode(const Telemetry& telemetry);

  void DrainLogs();

  void Receive();

  bool Receive(Transport& transport, ControlSource source);

  ControllerType& controller_;
  Transport& transport_;
  Transport& manual_transport_;
  NetworkEmulator inbound_;
  NetworkEmulator outbound_;
  std::multimap<TimePoint, std::function<void()>> tasks_;
  Logger* logger_ = nullptr;
  RecordSink record_sink_;
};

// -----------------------------------------------------------------------------

template <typename ControllerType, typename Transport>
SimulationRunner<ControllerType, Transport>::SimulationRunner(
    ControllerType& controller, Transport& transport,
    Transport& manual_transport, NetworkEmulator inbound,
    NetworkEmulator outbound)
    : controller_(controller),
      transport_(transport),
      manual_transport_(manual_transport),
      inbound_(std::move(inbound)),
      outbound_(std::move(outbound)) {}

template <typename ControllerType, typename Transport>
SimulationRunner<ControllerType, Transport>::SimulationRunner(
    ControllerType& controller, Transport& transport,
    Transport& manual_transport, NetworkEmulator inbound,
    NetworkEmulator outbound, Logger& logger, RecordSink record_sink)
    : controller_(controller),
      transport_(transport),
      manual_transport_(manual_transport),
      inbound_(std::move(inbound)),
      outbound_(std::move(outbound)),
      logger_(&logger),
      record_sink_(std::move(record_sink)) {}

template <typename ControllerType, typename Transport>
void SimulationRunner<ControllerType, Transport>::RunOnce() {
  DrainLogs();
  Receive();
  const auto deadline =
      tasks_.empty() ? TimePoint::max() : tasks_.begin()->first;
  const auto telemetry =
      controller_.Process(std::max(controller_.GetTime(), deadline));
  DrainLogs();
  if (telemetry.has_telemetry) {
    for (auto& message : Encode(telemetry)) {
      if (auto delivery = outbound_.Process(std::move(message))) {
        tasks_.emplace(
            delivery->arrival_time,
            [&transport = transport_, message = std::move(delivery->message)] {
              transport.Send(message.data);
            });
      }
    }
    return;
  }
  assert(!tasks_.empty());
  auto task = tasks_.begin();
  std::this_thread::sleep_until(task->first);
  auto callback = std::move(task->second);
  tasks_.erase(task);
  callback();
  DrainLogs();
}

template <typename ControllerType, typename Transport>
template <typename StopRequested>
void SimulationRunner<ControllerType, Transport>::Run(
    StopRequested stop_requested) {
  while (!stop_requested()) {
    RunOnce();
  }
  DrainLogs();
}

template <typename ControllerType, typename Transport>
std::vector<NetworkMessage> SimulationRunner<ControllerType, Transport>::Encode(
    const Telemetry& telemetry) {
  std::vector<NetworkMessage> messages;
  if (telemetry.heartbeat) {
    messages.emplace_back(*telemetry.heartbeat);
  }
  if (telemetry.attitude) {
    messages.emplace_back(*telemetry.attitude);
  }
  if (telemetry.global_position) {
    messages.emplace_back(*telemetry.global_position);
  }
  if (telemetry.battery_status) {
    messages.emplace_back(*telemetry.battery_status);
  }
  return messages;
}

template <typename ControllerType, typename Transport>
void SimulationRunner<ControllerType, Transport>::DrainLogs() {
  if (logger_ == nullptr) {
    return;
  }
  for (const auto& record : logger_->TakeRecords()) {
    if (!record_sink_) {
      continue;
    }
    try {
      record_sink_(record);
    } catch (const std::exception& error) {
      std::cerr << "syslog: " << error.what() << '\n';
    }
  }
}

template <typename ControllerType, typename Transport>
void SimulationRunner<ControllerType, Transport>::Receive() {
  while (true) {
    const bool automatic = Receive(transport_, ControlSource::kAutomatic);
    const bool manual = Receive(manual_transport_, ControlSource::kManual);
    if (!automatic && !manual) {
      break;
    }
  }
}

template <typename ControllerType, typename Transport>
bool SimulationRunner<ControllerType, Transport>::Receive(
    Transport& transport, ControlSource source) {
  auto bytes = transport.Receive();
  if (!bytes) {
    return false;
  }
  NetworkMessage message(std::move(*bytes));
  const std::vector<std::byte> payload(message.data.begin(),
                                       message.data.end() - 1);
  std::function<void()> callback;
  switch (message.type) {
    case MessageType::kCommand: {
      const auto command = Deserialize<CommandMessage>(payload);
      callback = [&controller = controller_, command] {
        controller.Command(command.command);
      };
      break;
    }
    case MessageType::kControl: {
      const auto command = Deserialize<ControlMessage>(payload);
      callback = [&controller = controller_, command, source] {
        if (source == ControlSource::kManual) {
          controller.SetManualControl(command.position, command.orientation);
        } else {
          controller.SetAutomaticControl(command.position, command.orientation);
        }
      };
      break;
    }
    default:
      throw std::invalid_argument("Expected controller command");
  }
  if (auto delivery = inbound_.Process(std::move(message))) {
    tasks_.emplace(delivery->arrival_time, std::move(callback));
  }
  return true;
}
