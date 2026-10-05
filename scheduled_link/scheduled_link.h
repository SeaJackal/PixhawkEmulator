#pragma once

#include "network_emulator.h"

#include <map>
#include <optional>
#include <vector>

class ScheduledLink {
 public:
  explicit ScheduledLink(NetworkEmulator emulator);

  bool Submit(NetworkMessage message);
  [[nodiscard]] std::optional<simulation::TimePoint> NextDeadline() const;
  std::vector<NetworkMessage> TakeDue(simulation::TimePoint until);
  std::optional<NetworkMessageInfo> TakeNextDue(simulation::TimePoint until);

 private:
  NetworkEmulator emulator_;
  std::multimap<simulation::TimePoint, NetworkMessage> pending_;
};
