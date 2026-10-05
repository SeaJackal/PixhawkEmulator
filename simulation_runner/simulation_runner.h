#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <thread>
#include <utility>

#include "controller_protocol.h"
#include "scheduled_link.h"

template <typename ControllerType, typename Transport,
          typename Clock = std::chrono::steady_clock>
class SimulationRunner {
 public:
  using TimePoint = simulation::TimePoint;

  SimulationRunner(ControllerType& controller, Transport& transport,
                   Clock& clock, ScheduledLink inbound, ScheduledLink outbound)
      : controller_(controller),
        transport_(transport),
        clock_(clock),
        inbound_(std::move(inbound)),
        outbound_(std::move(outbound)) {}

  void RunOnce() {
    Receive();
    const auto telemetry =
        controller_.Process(std::max(controller_.GetTime(), NextDeadline()));
    if (telemetry.has_telemetry) {
      for (auto& message : ControllerProtocol::Encode(telemetry)) {
        outbound_.Submit(std::move(message));
      }
    }
    std::this_thread::sleep_until(telemetry.timestamp);
    Flush();
    while (auto delivery = inbound_.TakeNextDue(clock_.now())) {
      ControllerProtocol::Deliver(controller_, delivery->message);
    }
  }

  template <typename StopRequested>
  void Run(StopRequested stop_requested) {
    while (!stop_requested()) {
      RunOnce();
    }
  }

 private:
  TimePoint NextDeadline() const {
    auto next = TimePoint::max();
    if (const auto deadline = inbound_.NextDeadline()) {
      next = std::min(next, *deadline);
    }
    if (const auto deadline = outbound_.NextDeadline()) {
      next = std::min(next, *deadline);
    }
    return next;
  }

  void Receive() {
    while (true) {
      auto bytes = transport_.Receive();
      if (!bytes) break;
      NetworkMessage message(std::move(*bytes));
      ControllerProtocol::Decode(message);
      inbound_.Submit(std::move(message));
    }
  }

  void Flush() {
    while (auto delivery = outbound_.TakeNextDue(clock_.now())) {
      transport_.Send(delivery->message.data);
    }
  }

  ControllerType& controller_;
  Transport& transport_;
  Clock& clock_;
  ScheduledLink inbound_;
  ScheduledLink outbound_;
};
