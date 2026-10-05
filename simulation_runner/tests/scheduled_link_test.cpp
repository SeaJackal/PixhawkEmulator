#include "scheduled_link.h"

#include <gtest/gtest.h>

namespace {

using namespace std::chrono_literals;
using Command = ControllerStateMachine::CommandType;
const simulation::TimePoint start{};

TEST(ScheduledLinkTest, OrdersByArrivalAndPreservesInsertionOrderForTies) {
  NetworkEmulator emulator{5ms};
  emulator.AddDelayFilter(start, start + 10ms, MessageType::kCommand, 30ms);
  ScheduledLink link{emulator};
  EXPECT_FALSE(link.NextDeadline());
  link.Submit(NetworkMessage{CommandMessage{start, Command::kUnguard}});
  link.Submit(NetworkMessage{CommandMessage{start + 10ms, Command::kToManual}});
  link.Submit(NetworkMessage{CommandMessage{start + 10ms, Command::kHold}});
  EXPECT_EQ(link.NextDeadline(), start + 15ms);
  EXPECT_TRUE(link.TakeDue(start + 14ms).empty());
  const auto due = link.TakeDue(start + 15ms);
  ASSERT_EQ(due.size(), 2u);
  EXPECT_EQ(due[0].data.front(), static_cast<std::byte>(Command::kToManual));
  EXPECT_EQ(due[1].data.front(), static_cast<std::byte>(Command::kHold));
  EXPECT_EQ(link.NextDeadline(), start + 35ms);
  EXPECT_EQ(link.TakeDue(start + 1s).size(), 1u);
  EXPECT_FALSE(link.NextDeadline());
}

TEST(ScheduledLinkTest, DropsMessagesAndSupportsZeroLatency) {
  NetworkEmulator emulator{0ms};
  emulator.AddDropFilter(start, start + 10ms, MessageType::kCommand);
  ScheduledLink link{emulator};
  EXPECT_FALSE(link.Submit(NetworkMessage{CommandMessage{start, Command::kUnguard}}));
  EXPECT_FALSE(link.NextDeadline());
  EXPECT_TRUE(link.Submit(NetworkMessage{CommandMessage{start + 10ms, Command::kUnguard}}));
  EXPECT_EQ(link.TakeDue(start + 10ms).size(), 1u);
}

TEST(ScheduledLinkTest, RejectsOverflowingArrivalTimesWithoutQueuing) {
  ScheduledLink link{NetworkEmulator{10ms}};
  const auto near_max = simulation::TimePoint::max() - 5ms;
  EXPECT_THROW(link.Submit(NetworkMessage{CommandMessage{near_max, Command::kUnguard}}),
               std::invalid_argument);
  EXPECT_FALSE(link.NextDeadline());
  NetworkEmulator emulator{1ms};
  emulator.AddDelayFilter(near_max, simulation::TimePoint::max(), MessageType::kCommand, 10ms);
  ScheduledLink filtered{emulator};
  EXPECT_THROW(filtered.Submit(NetworkMessage{CommandMessage{near_max, Command::kUnguard}}),
               std::invalid_argument);
  EXPECT_FALSE(filtered.NextDeadline());
}

}
