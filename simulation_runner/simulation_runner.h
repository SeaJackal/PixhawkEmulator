#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <thread>
#include <utility>

#include "controller_protocol.h"
#include "logger.h"
#include "network_emulator.h"

template <typename ControllerType, typename Transport>
class SimulationRunner {
 public:
  using TimePoint = simulation::TimePoint;
  using RecordSink = std::function<void(const Logger::Record&)>;

  SimulationRunner(ControllerType& controller, Transport& transport,
                   NetworkEmulator inbound, NetworkEmulator outbound)
      : controller_(controller),
        transport_(transport),
        inbound_(std::move(inbound)),
        outbound_(std::move(outbound)) {}

  SimulationRunner(ControllerType& controller, Transport& transport,
                   NetworkEmulator inbound, NetworkEmulator outbound,
                   Logger& logger, RecordSink record_sink = {})
      : SimulationRunner(controller, transport, std::move(inbound),
                         std::move(outbound)) {
    logger_ = &logger;
    record_sink_ = std::move(record_sink);
  }

  void RunOnce() {
    DrainLogs();
    Receive();
    const auto deadline =
        tasks_.empty() ? TimePoint::max() : tasks_.begin()->first;
    const auto telemetry =
        controller_.Process(std::max(controller_.GetTime(), deadline));
    DrainLogs();
    if (telemetry.has_telemetry) {
      for (auto& message : ControllerProtocol::Encode(telemetry)) {
        if (auto delivery = outbound_.Process(std::move(message))) {
          tasks_.emplace(delivery->arrival_time,
                         [&transport = transport_,
                          message = std::move(delivery->message)] {
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

  template <typename StopRequested>
  void Run(StopRequested stop_requested) {
    while (!stop_requested()) {
      RunOnce();
    }
    DrainLogs();
  }

 private:
  void DrainLogs() {
    if (!logger_) return;
    for (const auto& record : logger_->TakeRecords()) {
      if (!record_sink_) continue;
      try {
        record_sink_(record);
      } catch (const std::exception& error) {
        std::cerr << "syslog: " << error.what() << '\n';
      }
    }
  }

  void Receive() {
    while (true) {
      auto bytes = transport_.Receive();
      if (!bytes) break;
      NetworkMessage message(std::move(*bytes));
      ControllerProtocol::Decode(message);
      if (auto delivery = inbound_.Process(std::move(message))) {
        tasks_.emplace(delivery->arrival_time,
                       [&controller = controller_,
                        message = std::move(delivery->message)] {
                         ControllerProtocol::Deliver(controller, message);
                       });
      }
    }
  }

  ControllerType& controller_;
  Transport& transport_;
  NetworkEmulator inbound_;
  NetworkEmulator outbound_;
  std::multimap<TimePoint, std::function<void()>> tasks_;
  Logger* logger_ = nullptr;
  RecordSink record_sink_;
};
