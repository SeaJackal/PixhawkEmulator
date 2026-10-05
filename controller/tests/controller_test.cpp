#include "controller.h"

#include <gtest/gtest.h>

#include <vector>

namespace {

using namespace std::chrono_literals;
using State = Controller<>::State;
using Command = Controller<>::CommandType;

class ControllerTelemetryTest : public ::testing::Test {
 protected:
  const simulation::TimePoint start{42s};
  DynamicModel model;
  BatteryModel battery;
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start};
  Controller<> controller{model, battery, state_machine, start};

  Telemetry Steps(int count) {
    Telemetry telemetry;
    for (int i = 0; i < count; ++i) {
      telemetry = controller.Process();
    }
    return telemetry;
  }

  void Manual() {
    ASSERT_TRUE(controller.Command(Command::kUnguard));
    Steps(50);
    ASSERT_TRUE(controller.Command(Command::kToManual));
  }
};

TEST_F(ControllerTelemetryTest, ReturnsOnlyMessagesDueAtEachFixedStep) {
  EXPECT_EQ(Controller<>::kProcessPeriod, 20ms);
  for (int tick = 1; tick <= 101; ++tick) {
    const auto telemetry = controller.Process();
    const auto timestamp = start + tick * 20ms;
    ASSERT_TRUE(telemetry.attitude);
    EXPECT_EQ(telemetry.attitude->timestamp, timestamp);
    EXPECT_EQ(telemetry.heartbeat.has_value(), tick % 50 == 0);
    EXPECT_EQ(telemetry.global_position.has_value(), tick % 5 == 0);
    EXPECT_EQ(telemetry.battery_status.has_value(), tick % 25 == 0);
    if (telemetry.heartbeat) {
      EXPECT_EQ(telemetry.heartbeat->timestamp, timestamp);
      EXPECT_EQ(telemetry.heartbeat->state, State::kGuarded);
      EXPECT_FALSE(telemetry.heartbeat->ready);
    }
    if (telemetry.global_position) {
      EXPECT_EQ(telemetry.global_position->timestamp, timestamp);
    }
    if (telemetry.battery_status) {
      EXPECT_EQ(telemetry.battery_status->timestamp, timestamp);
      EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 1.0);
      EXPECT_NEAR(telemetry.battery_status->charge_amp_hours,
                  2.0 - tick * 0.02 / 3600.0, 1e-11);
    }
  }
}

TEST_F(ControllerTelemetryTest, ForwardsCommandsAndUsesExternalLogger) {
  EXPECT_FALSE(controller.SetManualControl({}, {}));
  EXPECT_FALSE(controller.SetAutomaticControl({}, {}));
  EXPECT_FALSE(controller.Command(Command::kHold));
  EXPECT_NE(logger.GetText().find("rejected manual target in guarded"),
            std::string::npos);
  Manual();
  EXPECT_EQ(state_machine.GetState(), State::kManual);
  EXPECT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {}));
  EXPECT_FALSE(controller.SetAutomaticControl({}, {}));
  EXPECT_TRUE(controller.Command(Command::kToAutomatic));
  EXPECT_TRUE(controller.SetAutomaticControl({}, {}));
  EXPECT_FALSE(controller.SetManualControl({}, {}));
  EXPECT_TRUE(controller.Command(Command::kHold));
  EXPECT_TRUE(controller.Command(Command::kLand));
  EXPECT_EQ(state_machine.GetState(), State::kReady);
}

TEST_F(ControllerTelemetryTest, ReadinessCoversAllStates) {
  auto telemetry = Steps(50);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_FALSE(telemetry.heartbeat->ready);
  Steps(25);
  controller.Command(Command::kUnguard);
  telemetry = Steps(25);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_EQ(telemetry.heartbeat->state, State::kNotReady);
  EXPECT_FALSE(telemetry.heartbeat->ready);
  telemetry = Steps(50);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_EQ(telemetry.heartbeat->state, State::kReady);
  EXPECT_TRUE(telemetry.heartbeat->ready);
  controller.Command(Command::kToManual);
  telemetry = Steps(50);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_EQ(telemetry.heartbeat->state, State::kManual);
  EXPECT_TRUE(telemetry.heartbeat->ready);
  Steps(40);
  controller.Command(Command::kToAutomatic);
  telemetry = Steps(10);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_EQ(telemetry.heartbeat->state, State::kAutomatic);
  EXPECT_TRUE(telemetry.heartbeat->ready);
  telemetry = Steps(50);
  ASSERT_TRUE(telemetry.heartbeat);
  EXPECT_EQ(telemetry.heartbeat->state, State::kHolding);
  EXPECT_TRUE(telemetry.heartbeat->ready);
}

TEST_F(ControllerTelemetryTest, IntegratesArrivalWithinStepBeforeSampling) {
  Manual();
  controller.SetManualControl({0.125, 0.0, 0.0}, {0.0, -0.125, 0.0});
  auto telemetry = Steps(5);
  ASSERT_TRUE(telemetry.global_position);
  ASSERT_TRUE(telemetry.attitude);
  EXPECT_NEAR(telemetry.global_position->position.x_meters, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second, 1.0);
  EXPECT_NEAR(telemetry.attitude->orientation.pitch_radians, -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(
      telemetry.attitude->angular_velocity.pitch_radians_per_second, -1.0);
  telemetry = Steps(20);
  ASSERT_TRUE(telemetry.global_position);
  ASSERT_TRUE(telemetry.attitude);
  ASSERT_TRUE(telemetry.battery_status);
  EXPECT_EQ(telemetry.attitude->timestamp, start + 1500ms);
  EXPECT_DOUBLE_EQ(telemetry.global_position->position.x_meters, 0.125);
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(telemetry.attitude->orientation.pitch_radians, -0.125);
  EXPECT_DOUBLE_EQ(
      telemetry.attitude->angular_velocity.pitch_radians_per_second, 0.0);
  const double charge = 2.0 - (1.0 + 10.0 * 0.125 + 0.375) / 3600.0;
  EXPECT_NEAR(telemetry.battery_status->charge_amp_hours, charge, 1e-11);
  EXPECT_NEAR(telemetry.battery_status->voltage_volts,
              9.0 + 3.6 * charge / 2.0, 1e-11);
  EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 1.0);
}

TEST_F(ControllerTelemetryTest, SamplesStateAndMotionAfterCoincidentTimeout) {
  Manual();
  Steps(25);
  controller.Command(Command::kToAutomatic);
  controller.SetAutomaticControl({10.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
  const auto telemetry = Steps(25);
  ASSERT_TRUE(telemetry.heartbeat);
  ASSERT_TRUE(telemetry.global_position);
  ASSERT_TRUE(telemetry.attitude);
  ASSERT_TRUE(telemetry.battery_status);
  EXPECT_EQ(telemetry.heartbeat->timestamp, start + 2s);
  EXPECT_EQ(telemetry.heartbeat->state, State::kHolding);
  EXPECT_NEAR(telemetry.global_position->position.x_meters, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(
      telemetry.attitude->angular_velocity.roll_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 1.0);
  EXPECT_NEAR(telemetry.battery_status->charge_amp_hours,
              2.0 - (1.5 + 10.0 * 0.5) / 3600.0, 1e-11);
}

TEST_F(ControllerTelemetryTest, HandlesZeroAndPartialStepReadinessTimeouts) {
  for (const auto timeout : {0ms, 7ms}) {
    ControllerStateMachine custom{model, logger, start, timeout};
    Controller local{model, battery, custom, start};
    local.Command(Command::kUnguard);
    const auto telemetry = local.Process();
    EXPECT_EQ(custom.GetState(), State::kReady);
    ASSERT_TRUE(telemetry.attitude);
    EXPECT_EQ(telemetry.attitude->timestamp, start + 20ms);
    EXPECT_FALSE(telemetry.heartbeat);
  }
}

TEST_F(ControllerTelemetryTest, ContinuesAtCriticalWithoutControllerLogging) {
  Manual();
  controller.SetManualControl({10000.0, 0.0, 0.0}, {});
  logger.Clear();
  const auto telemetry = Steps(35000);
  ASSERT_TRUE(telemetry.heartbeat);
  ASSERT_TRUE(telemetry.battery_status);
  EXPECT_EQ(telemetry.heartbeat->state, State::kManual);
  EXPECT_TRUE(telemetry.heartbeat->ready);
  EXPECT_DOUBLE_EQ(telemetry.battery_status->charge_amp_hours, 0.2);
  EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 10.0);
  EXPECT_TRUE(logger.GetText().empty());
}

class StubModel {
 public:
  double current = 7.0;
  double GetCurrentAmps() const { return current; }
  Orientation GetOrientation() const { return {1.0, 2.0, 3.0}; }
  AngularVelocity GetAngularVelocity() const { return {4.0, 5.0, 6.0}; }
  Position GetPosition() const { return {7.0, 8.0, 9.0}; }
  LinearVelocity GetLinearVelocity() const { return {10.0, 11.0, 12.0}; }
};

class RecordingBattery {
 public:
  std::vector<std::pair<double, simulation::Duration>> calls;
  bool Process(double current, simulation::Duration elapsed) {
    calls.emplace_back(current, elapsed);
    return true;
  }
  double GetVoltageVolts() const { return 11.5; }
  double GetChargeAmpHours() const { return 1.5; }
};

class StubStateMachine {
 public:
  using TimePoint = simulation::TimePoint;
  using CommandType = Controller<>::CommandType;

  StubStateMachine(StubModel& model, TimePoint start)
      : model_(model), start_(start) {}

  std::vector<TimePoint> endpoints;
  CommandType command = CommandType::kLand;
  Position target;
  Orientation orientation;
  bool Command(CommandType value) {
    command = value;
    return false;
  }
  bool SetManualControl(const Position& position, const Orientation& angles) {
    target = position;
    orientation = angles;
    return false;
  }
  bool SetAutomaticControl(const Position& position,
                           const Orientation& angles) {
    target = position;
    orientation = angles;
    return true;
  }
  State GetState() const { return State::kHolding; }
  TimePoint Process(TimePoint until) {
    endpoints.push_back(until);
    if (endpoints.size() == 1) {
      model_.current = 2.0;
      return start_ + 7ms;
    }
    return until;
  }

 private:
  StubModel& model_;
  TimePoint start_;
};

TEST(ControllerInjectionTest, UsesTemplateDependenciesAndIntegratesEachEvent) {
  const simulation::TimePoint start{123s};
  StubModel model;
  RecordingBattery battery;
  StubStateMachine state_machine{model, start};
  Controller<StubModel, RecordingBattery, StubStateMachine> controller{
      model, battery, state_machine, start};
  EXPECT_TRUE(state_machine.endpoints.empty());
  EXPECT_TRUE(battery.calls.empty());
  EXPECT_FALSE(controller.Command(Command::kToManual));
  EXPECT_EQ(state_machine.command, Command::kToManual);
  EXPECT_FALSE(controller.SetManualControl({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}));
  EXPECT_DOUBLE_EQ(state_machine.target.y_meters, 2.0);
  EXPECT_DOUBLE_EQ(state_machine.orientation.pitch_radians, 5.0);
  EXPECT_TRUE(controller.SetAutomaticControl({7.0, 8.0, 9.0}, {}));
  EXPECT_DOUBLE_EQ(state_machine.target.z_meters, 9.0);
  const auto telemetry = controller.Process();
  ASSERT_EQ(state_machine.endpoints.size(), 2u);
  EXPECT_EQ(state_machine.endpoints[0], start + 20ms);
  EXPECT_EQ(state_machine.endpoints[1], start + 20ms);
  ASSERT_EQ(battery.calls.size(), 2u);
  EXPECT_DOUBLE_EQ(battery.calls[0].first, 7.0);
  EXPECT_EQ(battery.calls[0].second, 7ms);
  EXPECT_DOUBLE_EQ(battery.calls[1].first, 2.0);
  EXPECT_EQ(battery.calls[1].second, 13ms);
  ASSERT_TRUE(telemetry.attitude);
  EXPECT_EQ(telemetry.attitude->timestamp, start + 20ms);
  EXPECT_DOUBLE_EQ(telemetry.attitude->orientation.yaw_radians, 3.0);
  EXPECT_DOUBLE_EQ(telemetry.attitude->angular_velocity.yaw_radians_per_second, 6.0);
  Telemetry heartbeat_telemetry;
  for (int tick = 1; tick < 50; ++tick) {
    heartbeat_telemetry = controller.Process();
  }
  ASSERT_TRUE(heartbeat_telemetry.heartbeat);
  EXPECT_EQ(heartbeat_telemetry.heartbeat->timestamp, start + 1s);
  EXPECT_EQ(heartbeat_telemetry.heartbeat->state, State::kHolding);
  EXPECT_TRUE(heartbeat_telemetry.heartbeat->ready);
  ASSERT_TRUE(heartbeat_telemetry.global_position);
  EXPECT_DOUBLE_EQ(heartbeat_telemetry.global_position->position.z_meters, 9.0);
  EXPECT_DOUBLE_EQ(
      heartbeat_telemetry.global_position->velocity.z_meters_per_second, 12.0);
  ASSERT_TRUE(heartbeat_telemetry.battery_status);
  EXPECT_DOUBLE_EQ(heartbeat_telemetry.battery_status->voltage_volts, 11.5);
  EXPECT_DOUBLE_EQ(heartbeat_telemetry.battery_status->charge_amp_hours, 1.5);
  EXPECT_DOUBLE_EQ(heartbeat_telemetry.battery_status->current_amps, 2.0);
}

}
