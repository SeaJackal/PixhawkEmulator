#pragma once

#include <optional>
#include <vector>

#include "message_type.h"
#include "network_message.h"
#include "simulation_time.h"

struct NetworkMessageInfo {
  NetworkMessage message;
  simulation::TimePoint arrival_time;
};

class NetworkEmulator {
 public:
  using TimePoint = simulation::TimePoint;
  using Duration = simulation::Duration;

  explicit NetworkEmulator(Duration latency);

  [[nodiscard]] std::optional<NetworkMessageInfo> Process(
      NetworkMessage message) const;
  void AddDelayFilter(TimePoint start, TimePoint end, MessageType type,
                      Duration delay);
  void AddDropFilter(TimePoint start, TimePoint end, MessageType type);

 private:
  enum class FilterKind { kDelay, kDrop };

  struct Filter {
    TimePoint start;
    TimePoint end;
    MessageType type;
    FilterKind kind;
    Duration delay;
  };

  Duration latency_;
  std::vector<Filter> filters_;
};
