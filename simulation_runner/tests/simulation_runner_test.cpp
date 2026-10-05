#include "controller.h"
#include "simulation_runner.h"

#include <gtest/gtest.h>

#include <limits>
#include <deque>
#include <system_error>
#include <functional>

namespace {

using namespace std::chrono_literals;
using TimePoint = simulation::TimePoint;
using Command = ControllerStateMachine::CommandType;
using State = ControllerStateMachine::State;

struct FakeClock {
  TimePoint time{100s};
  TimePoint now() const { return time; }
};

struct FakeTransport {
  std::deque<std::vector<std::byte>> incoming;
  std::vector<NetworkMessage> sent;
  bool blocked = false;
  bool fatal = false;
  std::function<void()> on_send;
  std::size_t attempts = 0;

  std::optional<std::vector<std::byte>> Receive() {
    if (fatal) throw std::runtime_error("Socket failure");
    if (incoming.empty()) return std::nullopt;
    auto bytes = std::move(incoming.front());
    incoming.pop_front();
    return bytes;
  }

  void Send(const std::vector<std::byte>& bytes) {
    ++attempts;
    if (blocked) throw std::system_error(
        std::make_error_code(std::errc::operation_would_block), "sendto");
    sent.emplace_back(bytes);
    if (on_send) on_send();
  }

  template <typename Message>
  void Push(const Message& message) {
    incoming.push_back(NetworkMessage{message}.data);
  }
};

class RunnerTest : public ::testing::Test {
 protected:
  TimePoint start{42s};
  DynamicModel model;
  BatteryModel battery;
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start, 0ms};
  Controller<> controller{model, battery, state_machine, start};
  FakeClock clock{start};
  FakeTransport transport;

  auto Runner(NetworkEmulator inbound = NetworkEmulator{0ms},
              NetworkEmulator outbound = NetworkEmulator{0ms}) {
    return SimulationRunner{controller, transport, clock,
                            ScheduledLink{inbound}, ScheduledLink{outbound}};
  }

  void Manual() {
    ASSERT_TRUE(controller.Command(Command::kUnguard));
    controller.Process(start + 1ms);
    clock.time = start + 1ms;
    ASSERT_TRUE(controller.Command(Command::kToManual));
  }
};

TEST_F(RunnerTest, AdvancesThenWaitsBeforeDeliveringCommandBetweenTicks) {
  Manual();
  auto runner = Runner(NetworkEmulator{3ms});
  transport.Push(ManualControlMessage{start + 4ms, {1.0, 0.0, 0.0}, {}});
  clock.time += 6ms;
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 7ms);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1.0);
  clock.time += 13ms;
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.013, 1e-12);
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(transport.sent.front().timestamp, start + 20ms);
}

TEST_F(RunnerTest, SamplesBeforeEqualTimeCommandsAndPreservesTheirOrder) {
  auto runner = Runner();
  transport.Push(CommandMessage{start + 1s, Command::kUnguard});
  transport.Push(CommandMessage{start + 1s, Command::kHold});
  transport.on_send = [&] { EXPECT_EQ(state_machine.GetState(), State::kGuarded); };
  for (int i = 0; i < 50; ++i) {
    clock.time += 20ms;
    runner.RunOnce();
  }
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  auto heartbeat = std::find_if(transport.sent.begin(), transport.sent.end(),
      [](const auto& message) { return message.type == MessageType::kHeartbeat; });
  ASSERT_NE(heartbeat, transport.sent.end());
  auto bytes = heartbeat->data;
  bytes.pop_back();
  EXPECT_EQ(Deserialize<HeartbeatMessage>(bytes).state, State::kGuarded);
  EXPECT_EQ(transport.sent.size(), 63u);
}

TEST_F(RunnerTest, ReordersCommandsByDeadlineAndAppliesLateCommandsWithoutRewind) {
  Manual();
  NetworkEmulator emulator{0ms};
  emulator.AddDelayFilter(start, start + 5ms, MessageType::kManualControl, 10ms);
  auto runner = Runner(emulator);
  transport.Push(ManualControlMessage{start + 2ms, {1.0, 0.0, 0.0}, {}});
  transport.Push(ManualControlMessage{start + 6ms, {-1.0, 0.0, 0.0}, {}});
  for (const auto elapsed : {5ms, 6ms, 8ms}) {
    clock.time += elapsed;
    runner.RunOnce();
  }
  EXPECT_NEAR(model.GetPosition().x_meters, 0.002, 1e-12);
  transport.Push(CommandMessage{start + 1ms, Command::kHold});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(state_machine.GetState(), State::kHolding);
}

TEST_F(RunnerTest, DelaysAndDropsTelemetryIndependentlyOfCommands) {
  NetworkEmulator inbound{0ms};
  inbound.AddDropFilter(start, start + 1s, MessageType::kCommand);
  NetworkEmulator outbound{5ms};
  outbound.AddDelayFilter(start, start + 30ms, MessageType::kAttitude, 30ms);
  outbound.AddDropFilter(start + 50ms, start + 1s, MessageType::kAttitude);
  auto runner = Runner(inbound, outbound);
  transport.Push(CommandMessage{start + 1ms, Command::kUnguard});
  clock.time += 20ms;
  runner.RunOnce();
  clock.time += 20ms;
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  EXPECT_TRUE(transport.sent.empty());
  clock.time += 5ms;
  runner.RunOnce();
  clock.time += 10ms;
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 2u);
  EXPECT_EQ(transport.sent[0].timestamp, start + 40ms);
  EXPECT_EQ(transport.sent[1].timestamp, start + 20ms);
  clock.time += 5ms;
  runner.RunOnce();
  clock.time += 20ms;
  runner.RunOnce();
  EXPECT_EQ(transport.sent.size(), 2u);
}

TEST_F(RunnerTest, PropagatesBlockedSendWithoutRetry) {
  auto runner = Runner();
  transport.blocked = true;
  clock.time += 20ms;
  EXPECT_THROW(runner.RunOnce(), std::system_error);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(transport.attempts, 1u);
  EXPECT_TRUE(transport.sent.empty());
  transport.blocked = false;
  clock.time += 20ms;
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(transport.sent.front().timestamp, start + 40ms);
}

TEST_F(RunnerTest, PropagatesMalformedUnexpectedAndNonFiniteInput) {
  auto runner = Runner();
  transport.incoming.push_back({});
  transport.incoming.push_back({std::byte{255}});
  transport.Push(AttitudeMessage{start, {}, {}});
  transport.Push(ManualControlMessage{start, {std::numeric_limits<double>::infinity(), 0, 0}, {}});
  for (int i = 0; i < 4; ++i) {
    EXPECT_THROW(runner.RunOnce(), std::invalid_argument);
    EXPECT_EQ(controller.GetTime(), start);
    EXPECT_TRUE(transport.sent.empty());
  }
  EXPECT_TRUE(transport.incoming.empty());
}

TEST_F(RunnerTest, ProcessesOnlyOneControllerEventPerIterationAndPropagatesFatalErrors) {
  auto runner = Runner();
  clock.time += 10s;
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 40ms);
  EXPECT_EQ(transport.sent.size(), 2u);
  transport.fatal = true;
  EXPECT_THROW(runner.RunOnce(), std::runtime_error);
}

TEST_F(RunnerTest, ProtocolDispatchesAutomaticControlsAndPreservesStateOnRejection) {
  Manual();
  auto runner = Runner();
  transport.Push(CommandMessage{start + 2ms, Command::kToAutomatic});
  transport.Push(AutomaticControlMessage{start + 3ms, {2.0, 0.0, 0.0}, {}});
  transport.Push(ManualControlMessage{start + 4ms, {-1.0, 0.0, 0.0}, {}});
  for (const auto elapsed : {1ms, 1ms, 1ms, 16ms}) {
    clock.time += elapsed;
    runner.RunOnce();
  }
  EXPECT_EQ(state_machine.GetState(), State::kAutomatic);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.017, 1e-12);
}

TEST_F(RunnerTest, AppliesEqualTimeTargetsInReceiveOrder) {
  Manual();
  auto runner = Runner();
  transport.Push(ManualControlMessage{start + 7ms, {1.0, 0.0, 0.0}, {}});
  transport.Push(ManualControlMessage{start + 7ms, {-1.0, 0.0, 0.0}, {}});
  clock.time += 6ms;
  runner.RunOnce();
  clock.time += 13ms;
  runner.RunOnce();
  EXPECT_NEAR(model.GetPosition().x_meters, -0.013, 1e-12);
}

TEST_F(RunnerTest, RejectsUnrepresentableArrivalsAndHandlesVeryOldCommands) {
  auto runner = Runner(NetworkEmulator{10ms});
  transport.Push(CommandMessage{TimePoint::max() - 1ms, Command::kUnguard});
  transport.Push(CommandMessage{TimePoint::min() + 1ms, Command::kUnguard});
  EXPECT_THROW(runner.RunOnce(), std::invalid_argument);
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_EQ(controller.GetTime(), start);
}

TEST_F(RunnerTest, SleepsUntilControllerSteadyClockTimestamp) {
  std::chrono::steady_clock real_clock;
  const auto before = real_clock.now();
  Logger local_logger{before};
  ControllerStateMachine local_state{model, local_logger, before};
  Controller local_controller{model, battery, local_state, before};
  SimulationRunner runner{local_controller, transport, real_clock,
      ScheduledLink{NetworkEmulator{0ms}}, ScheduledLink{NetworkEmulator{0ms}}};
  transport.on_send = [&] { EXPECT_GE(real_clock.now() - before, 20ms); };
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(local_controller.GetTime(), before + 20ms);
}

TEST_F(RunnerTest, OvershootDeliversByWallClockWithoutAdditionalControllerAdvancement) {
  Manual();
  auto runner = Runner(NetworkEmulator{0ms}, NetworkEmulator{5ms});
  transport.Push(ManualControlMessage{start + 30ms, {1, 0, 0}, {}});
  clock.time += 34ms;
  transport.on_send = [&] {
    EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0.0);
  };
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(clock.time, start + 35ms);
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1.0);
}

TEST_F(RunnerTest, InputArrivingAfterReceivePhaseIsReadOnNextIteration) {
  auto runner = Runner();
  transport.on_send = [&] {
    transport.Push(CommandMessage{start + 7ms, Command::kUnguard});
  };
  clock.time += 20ms;
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  EXPECT_EQ(transport.incoming.size(), 1u);
  transport.on_send = {};
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
}

TEST_F(RunnerTest, RefreshesWallClockWhileSendingOutput) {
  ScheduledLink outbound{NetworkEmulator{0ms}};
  outbound.Submit(NetworkMessage{AttitudeMessage{start + 20ms, {}, {}}});
  outbound.Submit(NetworkMessage{AttitudeMessage{start + 21ms, {}, {}}});
  SimulationRunner runner{controller, transport, clock,
      ScheduledLink{NetworkEmulator{0ms}}, std::move(outbound)};
  transport.on_send = [&] { clock.time += 1ms; };
  clock.time += 20ms;
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 3u);
  EXPECT_EQ(transport.sent.back().timestamp, start + 21ms);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
}

TEST_F(RunnerTest, DrainsInputAndOutputBeyondFormerBatchLimits) {
  ScheduledLink outbound{NetworkEmulator{0ms}};
  for (int i = 0; i < 300; ++i) {
    outbound.Submit(NetworkMessage{AttitudeMessage{start + 20ms, {}, {}}});
    transport.Push(CommandMessage{start + 20ms, Command::kHold});
  }
  SimulationRunner runner{controller, transport, clock,
      ScheduledLink{NetworkEmulator{0ms}}, std::move(outbound)};
  clock.time += 20ms;
  runner.RunOnce();
  EXPECT_TRUE(transport.incoming.empty());
  EXPECT_EQ(transport.sent.size(), 301u);
}

TEST_F(RunnerTest, SendExceptionStopsIterationBeforeCommandDelivery) {
  auto runner = Runner();
  transport.Push(CommandMessage{start + 20ms, Command::kUnguard});
  transport.blocked = true;
  clock.time += 20ms;
  EXPECT_THROW(runner.RunOnce(), std::system_error);
  EXPECT_EQ(transport.attempts, 1u);
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  EXPECT_TRUE(transport.sent.empty());
}

TEST_F(RunnerTest, RefreshesWallClockWhileDeliveringCommands) {
  struct TimedController : Controller<> {
    using Controller<>::Controller;
    FakeClock* clock = nullptr;
    bool Command(CommandType command) {
      clock->time += 1ms;
      return Controller<>::Command(command);
    }
  } timed{model, battery, state_machine, start};
  timed.clock = &clock;
  SimulationRunner runner{timed, transport, clock,
      ScheduledLink{NetworkEmulator{0ms}}, ScheduledLink{NetworkEmulator{0ms}}};
  transport.Push(CommandMessage{start + 20ms, Command::kUnguard});
  transport.Push(CommandMessage{start + 21ms, Command::kHold});
  clock.time += 20ms;
  runner.RunOnce();
  EXPECT_EQ(timed.GetTime(), start + 20ms);
  EXPECT_EQ(clock.time, start + 22ms);
}

TEST_F(RunnerTest, RunnerCreationDoesNotResetControllerTimeOrigin) {
  clock.time = start + 100ms;
  auto runner = Runner();
  transport.Push(CommandMessage{start + 30ms, Command::kUnguard});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(transport.sent.front().timestamp, start + 20ms);
}

}
