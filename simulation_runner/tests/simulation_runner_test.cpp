#include "controller.h"
#include "simulation_runner.h"

#include <gtest/gtest.h>

#include <deque>
#include <functional>
#include <limits>
#include <system_error>

namespace {

using namespace std::chrono_literals;
using TimePoint = simulation::TimePoint;
using Command = ControllerStateMachine::CommandType;
using State = ControllerStateMachine::State;

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
  BatteryModel battery{2.0, 2.0};
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start, 0ms};
  Controller<> controller{model, battery, state_machine, start};
  FakeTransport transport;
  FakeTransport manual_transport;

  auto Runner(NetworkEmulator inbound = NetworkEmulator{0ms},
              NetworkEmulator outbound = NetworkEmulator{0ms}) {
    return SimulationRunner{controller, transport, manual_transport, inbound, outbound};
  }

  void Manual() {
    ASSERT_TRUE(controller.Command(Command::kUnguard));
    controller.Process(start + 1ms);
    ASSERT_TRUE(controller.Command(Command::kToManual));
  }
};

TEST_F(RunnerTest, AdvancesBeforeCommandAndQueuesTelemetryBeforeSending) {
  Manual();
  auto runner = Runner(NetworkEmulator{3ms});
  manual_transport.Push(ControlMessage{start + 4ms, {1.0, 0.0, 0.0}, {}});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 7ms);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1.0);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.013, 1e-12);
  EXPECT_TRUE(transport.sent.empty());
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(transport.sent.front().timestamp, start + 20ms);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
}

TEST_F(RunnerTest, SamplesBeforeEqualTimeCommandsAndExecutesInInsertionOrder) {
  auto runner = Runner();
  transport.Push(CommandMessage{start + 1s, Command::kUnguard});
  transport.Push(CommandMessage{start + 1s, Command::kHold});
  for (int i = 0; i < 200 && controller.GetTime() < start + 1s; ++i) {
    runner.RunOnce();
  }
  ASSERT_EQ(controller.GetTime(), start + 1s);
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  const auto sent_before = transport.sent.size();
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_EQ(transport.sent.size(), sent_before);
  runner.RunOnce();
  EXPECT_EQ(transport.sent.size(), sent_before);
  transport.on_send = [&] {
    EXPECT_EQ(state_machine.GetState(), State::kNotReady);
    EXPECT_EQ(controller.GetTime(), start + 1s);
  };
  for (std::size_t i = 1; i <= 4; ++i) {
    runner.RunOnce();
    EXPECT_EQ(transport.sent.size(), sent_before + i);
  }
  ASSERT_EQ(transport.sent.size(), 63u);
  ASSERT_EQ(transport.sent[sent_before].type, MessageType::kHeartbeat);
  auto bytes = transport.sent[sent_before].data;
  bytes.pop_back();
  EXPECT_EQ(Deserialize<HeartbeatMessage>(bytes).state, State::kGuarded);
}

TEST_F(RunnerTest, ReordersCommandsByDeadlineAndAppliesLateCommandsWithoutRewind) {
  Manual();
  NetworkEmulator emulator{0ms};
  emulator.AddDelayFilter(start, start + 5ms, MessageType::kControl, 10ms);
  auto runner = Runner(emulator);
  manual_transport.Push(ControlMessage{start + 2ms, {1.0, 0.0, 0.0}, {}});
  manual_transport.Push(ControlMessage{start + 6ms, {-1.0, 0.0, 0.0}, {}});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 6ms);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 12ms);
  runner.RunOnce();
  EXPECT_NEAR(model.GetPosition().x_meters, 0.002, 1e-12);
  transport.Push(CommandMessage{start + 1ms, Command::kHold});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(state_machine.GetState(), State::kHolding);
  EXPECT_TRUE(transport.sent.empty());
}

TEST_F(RunnerTest, DelaysAndDropsTelemetryIndependentlyOfCommands) {
  NetworkEmulator inbound{0ms};
  inbound.AddDropFilter(start, start + 1s, MessageType::kCommand);
  NetworkEmulator outbound{5ms};
  outbound.AddDelayFilter(start, start + 30ms, MessageType::kAttitude, 30ms);
  outbound.AddDropFilter(start + 50ms, start + 1s, MessageType::kAttitude);
  auto runner = Runner(inbound, outbound);
  transport.Push(CommandMessage{start + 1ms, Command::kUnguard});
  runner.RunOnce();
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  EXPECT_TRUE(transport.sent.empty());
  transport.on_send = [&] {
    EXPECT_EQ(controller.GetTime(), transport.sent.size() == 1u
        ? start + 45ms : start + 55ms);
  };
  runner.RunOnce();
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 2u);
  EXPECT_EQ(transport.sent[0].timestamp, start + 40ms);
  EXPECT_EQ(transport.sent[1].timestamp, start + 20ms);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 60ms);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 80ms);
  EXPECT_EQ(transport.sent.size(), 2u);
}

TEST_F(RunnerTest, PropagatesBlockedSendWithoutRetry) {
  auto runner = Runner();
  transport.blocked = true;
  runner.RunOnce();
  EXPECT_EQ(transport.attempts, 0u);
  EXPECT_THROW(runner.RunOnce(), std::system_error);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(transport.attempts, 1u);
  transport.blocked = false;
  runner.RunOnce();
  EXPECT_TRUE(transport.sent.empty());
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
  EXPECT_EQ(transport.sent.front().timestamp, start + 40ms);
}

TEST_F(RunnerTest, PropagatesMalformedUnexpectedAndNonFiniteInput) {
  auto runner = Runner();
  transport.incoming.push_back({});
  transport.incoming.push_back({std::byte{255}});
  transport.Push(AttitudeMessage{start, {}, {}});
  manual_transport.Push(ControlMessage{start, {std::numeric_limits<double>::infinity(), 0, 0}, {}});
  for (int i = 0; i < 4; ++i) {
    EXPECT_THROW(runner.RunOnce(), std::invalid_argument);
    EXPECT_EQ(controller.GetTime(), start);
    EXPECT_TRUE(transport.sent.empty());
  }
  EXPECT_TRUE(transport.incoming.empty());
}

TEST_F(RunnerTest, EmptyQueueProcessesOneTelemetryEventAndPropagatesFatalErrors) {
  auto runner = Runner();
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_TRUE(transport.sent.empty());
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(transport.sent.size(), 1u);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 40ms);
  EXPECT_EQ(transport.sent.size(), 1u);
  transport.fatal = true;
  EXPECT_THROW(runner.RunOnce(), std::runtime_error);
}

TEST_F(RunnerTest, ProtocolDispatchesAutomaticControlsAndPreservesStateOnRejection) {
  Manual();
  auto runner = Runner();
  transport.Push(CommandMessage{start + 2ms, Command::kToAutomatic});
  transport.Push(ControlMessage{start + 3ms, {2.0, 0.0, 0.0}, {}});
  manual_transport.Push(ControlMessage{start + 4ms, {-1.0, 0.0, 0.0}, {}});
  for (int i = 0; i < 4; ++i) runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kAutomatic);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.017, 1e-12);
}

TEST_F(RunnerTest, IdenticalControlPacketUsesSourceAndRespectsCurrentMode) {
  Manual();
  const NetworkMessage message{ControlMessage{start + 2ms, {1, 0, 0}, {}}};
  EXPECT_FALSE(ControllerProtocol::Deliver(controller, message, ControlSource::kAutomatic));
  EXPECT_TRUE(ControllerProtocol::Deliver(controller, message, ControlSource::kManual));
  ASSERT_TRUE(controller.Command(Command::kToAutomatic));
  EXPECT_FALSE(ControllerProtocol::Deliver(controller, message, ControlSource::kManual));
  EXPECT_TRUE(ControllerProtocol::Deliver(controller, message, ControlSource::kAutomatic));
}

TEST_F(RunnerTest, PreservesBothSourcesThroughSharedControlDelay) {
  Manual();
  NetworkEmulator inbound{1ms};
  inbound.AddDelayFilter(start, start + 20ms, MessageType::kControl, 5ms);
  auto runner = Runner(inbound);
  transport.Push(ControlMessage{start + 2ms, {-1, 0, 0}, {}});
  manual_transport.Push(ControlMessage{start + 2ms, {1, 0, 0}, {}});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 8ms);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 8ms);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1);
  manual_transport.Push(CommandMessage{start + 8ms, Command::kToAutomatic});
  transport.Push(ControlMessage{start + 4ms, {-1, 0, 0}, {}});
  manual_transport.Push(ControlMessage{start + 4ms, {1, 0, 0}, {}});
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kAutomatic);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 10ms);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, -1);
  runner.RunOnce();
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, -1);
  EXPECT_TRUE(manual_transport.sent.empty());
}

TEST_F(RunnerTest, SharedControlDropAffectsBothPortsAndCommandsStillWork) {
  Manual();
  NetworkEmulator inbound{0ms};
  inbound.AddDropFilter(start, start + 1s, MessageType::kControl);
  auto runner = Runner(inbound);
  transport.Push(ControlMessage{start + 2ms, {-1, 0, 0}, {}});
  manual_transport.Push(ControlMessage{start + 2ms, {1, 0, 0}, {}});
  manual_transport.Push(CommandMessage{start + 3ms, Command::kToAutomatic});
  transport.Push(CommandMessage{start + 4ms, Command::kToManual});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 3ms);
  EXPECT_EQ(state_machine.GetState(), State::kAutomatic);
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kManual);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0);
  EXPECT_TRUE(transport.incoming.empty());
  EXPECT_TRUE(manual_transport.incoming.empty());
}

TEST_F(RunnerTest, AlternatesPortsStartingWithAutomaticForEqualTimeCommands) {
  Manual();
  auto runner = Runner();
  transport.Push(CommandMessage{start + 2ms, Command::kToAutomatic});
  transport.Push(CommandMessage{start + 2ms, Command::kToAutomatic});
  manual_transport.Push(CommandMessage{start + 2ms, Command::kToManual});
  manual_transport.Push(CommandMessage{start + 2ms, Command::kToManual});
  for (const auto expected : {State::kAutomatic, State::kManual,
                              State::kAutomatic, State::kManual}) {
    runner.RunOnce();
    EXPECT_EQ(controller.GetTime(), start + 2ms);
    EXPECT_EQ(state_machine.GetState(), expected);
  }
}

TEST_F(RunnerTest, AppliesOneEqualTimeTargetPerCallInReceiveOrder) {
  Manual();
  auto runner = Runner();
  manual_transport.Push(ControlMessage{start + 7ms, {1.0, 0.0, 0.0}, {}});
  manual_transport.Push(ControlMessage{start + 7ms, {-1.0, 0.0, 0.0}, {}});
  runner.RunOnce();
  EXPECT_TRUE(manual_transport.incoming.empty());
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1.0);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 7ms);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, -1.0);
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

TEST_F(RunnerTest, SleepsUntilScheduledSendDeadline) {
  const auto before = std::chrono::steady_clock::now();
  Logger local_logger{before};
  ControllerStateMachine local_state{model, local_logger, before};
  Controller local_controller{model, battery, local_state, before};
  SimulationRunner runner{local_controller, transport, manual_transport,
      NetworkEmulator{0ms}, NetworkEmulator{5ms}};
  transport.on_send = [&] {
    EXPECT_GE(std::chrono::steady_clock::now(), before + 25ms);
    EXPECT_EQ(local_controller.GetTime(), before + 25ms);
  };
  runner.RunOnce();
  EXPECT_TRUE(transport.sent.empty());
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
}

TEST_F(RunnerTest, ExecutesOverdueTasksChronologicallyWithControllerAdvancement) {
  Manual();
  auto runner = Runner(NetworkEmulator{0ms}, NetworkEmulator{5ms});
  manual_transport.Push(ControlMessage{start + 30ms, {1, 0, 0}, {}});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0.0);
  transport.on_send = [&] {
    EXPECT_EQ(controller.GetTime(), start + 25ms);
    EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0.0);
  };
  runner.RunOnce();
  ASSERT_EQ(transport.sent.size(), 1u);
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 30ms);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1.0);
}

TEST_F(RunnerTest, ReadsNewInputBeforeExecutingQueuedSend) {
  auto runner = Runner();
  runner.RunOnce();
  transport.Push(CommandMessage{start + 7ms, Command::kUnguard});
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_TRUE(transport.sent.empty());
  runner.RunOnce();
  EXPECT_EQ(transport.sent.size(), 1u);
}

TEST_F(RunnerTest, InputArrivingDuringCallbackIsReadOnNextIteration) {
  auto runner = Runner();
  transport.on_send = [&] {
    transport.Push(CommandMessage{start + 7ms, Command::kUnguard});
  };
  runner.RunOnce();
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  EXPECT_EQ(transport.incoming.size(), 1u);
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_EQ(controller.GetTime(), start + 20ms);
}

TEST_F(RunnerTest, DrainsAllInputButExecutesOnlyOneTaskPerCall) {
  auto runner = Runner();
  for (int i = 0; i < 300; ++i) {
    transport.Push(CommandMessage{start + 20ms, Command::kHold});
  }
  runner.RunOnce();
  EXPECT_TRUE(transport.incoming.empty());
  for (int i = 0; i < 300; ++i) {
    runner.RunOnce();
    EXPECT_TRUE(transport.sent.empty());
    EXPECT_EQ(controller.GetTime(), start + 20ms);
  }
  runner.RunOnce();
  EXPECT_EQ(transport.sent.size(), 1u);
}

TEST_F(RunnerTest, DroppedTelemetryReturnsBeforeExecutingQueuedCommand) {
  NetworkEmulator outbound{0ms};
  outbound.AddDropFilter(start, start + 1s, MessageType::kAttitude);
  auto runner = Runner(NetworkEmulator{0ms}, outbound);
  transport.Push(CommandMessage{start + 20ms, Command::kUnguard});
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 20ms);
  EXPECT_EQ(state_machine.GetState(), State::kGuarded);
  runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), State::kNotReady);
  EXPECT_TRUE(transport.sent.empty());
}

TEST_F(RunnerTest, ForwardsStartupAndCommandLogsWithoutDuplicates) {
  std::vector<Logger::Record> records;
  logger.Log(Logger::Level::kInfo, start, "startup");
  SimulationRunner runner{controller, transport, manual_transport, NetworkEmulator{0ms},
      NetworkEmulator{0ms}, logger,
      [&](const Logger::Record& record) { records.push_back(record); }};
  transport.Push(CommandMessage{start + 1ms, Command::kUnguard});
  runner.RunOnce();
  ASSERT_GE(records.size(), 2u);
  EXPECT_EQ(records.front().message, "startup");
  EXPECT_EQ(records.front().timestamp, start);
  EXPECT_EQ(records.back().timestamp, start + 1ms);
  EXPECT_FALSE(logger.GetText().empty());
  EXPECT_TRUE(logger.TakeRecords().empty());
  const auto count = records.size();
  runner.RunOnce();
  EXPECT_GT(records.size(), count);
  EXPECT_TRUE(logger.TakeRecords().empty());
  const auto after_process = records.size();
  runner.RunOnce();
  EXPECT_EQ(records.size(), after_process);
}

TEST_F(RunnerTest, ContinuesAfterLogSendFailureAndDoesNotRetry) {
  logger.Log(Logger::Level::kError, start, "first");
  logger.Log(Logger::Level::kInfo, start, "second");
  std::vector<std::string> attempts;
  SimulationRunner runner{controller, transport, manual_transport, NetworkEmulator{0ms},
      NetworkEmulator{0ms}, logger, [&](const Logger::Record& record) {
        attempts.push_back(record.message);
        if (attempts.size() == 1) {
          throw std::system_error(std::make_error_code(std::errc::operation_would_block));
        }
      }};
  testing::internal::CaptureStderr();
  runner.RunOnce();
  const auto diagnostic = testing::internal::GetCapturedStderr();
  EXPECT_NE(diagnostic.find("syslog:"), std::string::npos);
  EXPECT_EQ(attempts, (std::vector<std::string>{"first", "second"}));
  runner.RunOnce();
  EXPECT_EQ(attempts.size(), 2u);
  EXPECT_EQ(transport.sent.size(), 1u);
}

TEST_F(RunnerTest, FlushesLogsOnNormalExitEvenWithoutIterations) {
  logger.Log(Logger::Level::kInfo, start, "shutdown");
  std::vector<Logger::Record> records;
  SimulationRunner runner{controller, transport, manual_transport, NetworkEmulator{0ms},
      NetworkEmulator{0ms}, logger,
      [&](const Logger::Record& record) { records.push_back(record); }};
  runner.Run([] { return true; });
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records.front().message, "shutdown");
  runner.Run([] { return true; });
  EXPECT_EQ(records.size(), 1u);
}

TEST_F(RunnerTest, DisabledSinkDrainsRecordsAndPreservesText) {
  logger.Log(Logger::Level::kInfo, start, "local only");
  const auto text = logger.GetText();
  SimulationRunner runner{controller, transport, manual_transport, NetworkEmulator{0ms},
      NetworkEmulator{0ms}, logger};
  runner.RunOnce();
  EXPECT_TRUE(logger.TakeRecords().empty());
  EXPECT_EQ(logger.GetText(), text);
}

}
