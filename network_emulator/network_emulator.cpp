#include "network_emulator.h"

#include <cassert>
#include <optional>
#include <stdexcept>
#include <utility>

#include "message_type.h"
#include "network_message.h"
#include "simulation_time.h"

namespace {

simulation::TimePoint AddDelay(simulation::TimePoint timestamp,
                               simulation::Duration delay) {
  if (timestamp > simulation::TimePoint::max() - delay) {
    throw std::invalid_argument("Arrival time exceeds simulation clock range");
  }
  return timestamp + delay;
}

}

NetworkEmulator::NetworkEmulator(Duration latency) : latency_(latency) {
  assert(latency >= Duration::zero());
}

std::optional<NetworkMessageInfo> NetworkEmulator::Process(
    NetworkMessage message) const {
  auto arrival_time = AddDelay(message.timestamp, latency_);
  for (const auto& filter : filters_) {
    if (filter.type != message.type || message.timestamp < filter.start ||
        message.timestamp >= filter.end) {
      continue;
    }
    if (filter.kind == FilterKind::kDrop) {
      return std::nullopt;
    }
    arrival_time = AddDelay(arrival_time, filter.delay);
    break;
  }
  return NetworkMessageInfo{std::move(message), arrival_time};
}

void NetworkEmulator::AddDelayFilter(TimePoint start, TimePoint end,
                                     MessageType type, Duration delay) {
  assert(start <= end);
  assert(delay >= Duration::zero());
  filters_.push_back({start, end, type, FilterKind::kDelay, delay});
}

void NetworkEmulator::AddDropFilter(TimePoint start, TimePoint end,
                                    MessageType type) {
  assert(start <= end);
  filters_.push_back({start, end, type, FilterKind::kDrop, Duration::zero()});
}
