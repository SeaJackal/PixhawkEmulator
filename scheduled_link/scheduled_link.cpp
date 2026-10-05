#include "scheduled_link.h"

#include <utility>

ScheduledLink::ScheduledLink(NetworkEmulator emulator)
    : emulator_(std::move(emulator)) {}

bool ScheduledLink::Submit(NetworkMessage message) {
  auto result = emulator_.Process(std::move(message));
  if (!result) {
    return false;
  }
  pending_.emplace(result->arrival_time, std::move(result->message));
  return true;
}

std::optional<simulation::TimePoint> ScheduledLink::NextDeadline() const {
  if (pending_.empty()) {
    return std::nullopt;
  }
  return pending_.begin()->first;
}

std::vector<NetworkMessage> ScheduledLink::TakeDue(simulation::TimePoint until) {
  std::vector<NetworkMessage> messages;
  while (!pending_.empty() && pending_.begin()->first <= until) {
    auto entry = pending_.begin();
    messages.push_back(std::move(entry->second));
    pending_.erase(entry);
  }
  return messages;
}

std::optional<NetworkMessageInfo> ScheduledLink::TakeNextDue(simulation::TimePoint until) {
  if (pending_.empty() || pending_.begin()->first > until) return std::nullopt;
  const auto entry = pending_.begin();
  NetworkMessageInfo result{std::move(entry->second), entry->first};
  pending_.erase(entry);
  return result;
}
