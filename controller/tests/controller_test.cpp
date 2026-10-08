#include "controller.h"

#include <gtest/gtest-death-test.h>
#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <utility>
#include <vector>

#include "battery_model.h"
#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "logger.h"
#include "simulation_time.h"
#include "telemetry.h"

namespace {

using namespace std::chrono_literals;
using State = Controller<>::State;
using Command = Controller<>::CommandType;

class ControllerTelemetryTest : public ::testing::Test {
 protected:
  static constexpr simulation::TimePoint kStart{42s};
  DynamicModel model;
  BatteryModel battery{2.0, 2.0};
  Logger logger{kStart};
  ControllerStateMachine state_machine{model, logger, kStart};
  Controller<> controller{model, battery, state_machine, kStart};

  Telemetry Steps(int count) {
    Telemetry telemetry;
    for (int i = 0; i < count; ++i) {
      telemetry = controller.Process(controller.GetTime() + 20ms);
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
    const auto telemetry = controller.Process(controller.GetTime() + 20ms);
    const auto timestamp = kStart + tick * 20ms;
    EXPECT_EQ(telemetry.timestamp, timestamp);
    EXPECT_TRUE(telemetry.has_telemetry);
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
      EXPECT_TRUE(telemetry.global_position->gnss_valid);
    }
    if (telemetry.battery_status) {
      EXPECT_EQ(telemetry.battery_status->timestamp, timestamp);
      EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 1.0);
      EXPECT_NEAR(telemetry.battery_status->charge_amp_hours,
                  2.0 - tick * 0.02 / 3600.0, 1e-11);
    }
  }
}

TEST_F(ControllerTelemetryTest,
       GnssLossIntervalsUseSampleTimesAndPreserveCadence) {
  controller.AddGnssLoss(kStart + 700ms, 100ms);
  controller.AddGnssLoss(kStart + 200ms, 200ms);
  controller.AddGnssLoss(kStart + 300ms, 200ms);
  controller.AddGnssLoss(kStart + 500ms, 100ms);
  controller.AddGnssLoss(kStart + 110ms, 30ms);
  controller.AddGnssLoss(kStart + 100ms, 0ms);
  for (int tick = 1; tick <= 50; ++tick) {
    const auto telemetry = Steps(1);
    const auto elapsed = tick * 20ms;
    EXPECT_EQ(telemetry.timestamp, kStart + elapsed);
    EXPECT_TRUE(telemetry.attitude);
    EXPECT_EQ(telemetry.global_position.has_value(), tick % 5 == 0);
    EXPECT_EQ(telemetry.battery_status.has_value(), tick % 25 == 0);
    EXPECT_EQ(telemetry.heartbeat.has_value(), tick % 50 == 0);
    EXPECT_EQ(state_machine.GetState(), State::kGuarded);
    if (telemetry.global_position) {
      const bool lost =
          (200ms <= elapsed && elapsed < 600ms) || elapsed == 700ms;
      EXPECT_EQ(telemetry.global_position->gnss_valid, !lost);
    }
  }
}

TEST_F(ControllerTelemetryTest,
       GnssLossKeepsManualFlightAndModelTelemetryUpdating) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({10.0, 0.0, 0.0}, {}));
  const auto loss_start = controller.GetTime();
  controller.AddGnssLoss(loss_start, 300ms);
  for (int sample = 1; sample <= 4; ++sample) {
    const auto telemetry = Steps(5);
    ASSERT_TRUE(telemetry.global_position);
    EXPECT_EQ(telemetry.global_position->gnss_valid, sample >= 3);
    EXPECT_NEAR(telemetry.global_position->position.x_meters, sample * 0.1,
                1e-12);
    EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second,
                     1.0);
    EXPECT_EQ(state_machine.GetState(), State::kManual);
  }
}

TEST_F(ControllerTelemetryTest,
       GnssLossSupportsStartupAndAlreadyActiveIntervals) {
  controller.AddGnssLoss(kStart - 100ms, 300ms);
  auto telemetry = Steps(5);
  ASSERT_TRUE(telemetry.global_position);
  EXPECT_FALSE(telemetry.global_position->gnss_valid);
  telemetry = Steps(5);
  ASSERT_TRUE(telemetry.global_position);
  EXPECT_TRUE(telemetry.global_position->gnss_valid);
}

TEST_F(ControllerTelemetryTest, GnssLossRejectsInvalidRanges) {
  using TimePoint = simulation::TimePoint;
  using Duration = simulation::Duration;
  EXPECT_THROW(controller.AddGnssLoss(kStart, -1ns), std::invalid_argument);
  EXPECT_THROW(controller.AddGnssLoss(kStart, Duration::min()),
               std::invalid_argument);
  EXPECT_THROW(controller.AddGnssLoss(TimePoint::max(), 1ns),
               std::invalid_argument);
  EXPECT_THROW(controller.AddGnssLoss(kStart, Duration::max()),
               std::invalid_argument);
  EXPECT_NO_THROW(controller.AddGnssLoss(TimePoint::max(), 0ns));
  EXPECT_NO_THROW(controller.AddGnssLoss(TimePoint::max() - 1ns, 1ns));
  EXPECT_NO_THROW(controller.AddGnssLoss(TimePoint::min(), Duration::max()));
  const auto telemetry = Steps(5);
  ASSERT_TRUE(telemetry.global_position);
  EXPECT_TRUE(telemetry.global_position->gnss_valid);
}

TEST_F(ControllerTelemetryTest, PartialStepsPreserveAllTelemetryBoundaries) {
  const auto partial = controller.Process(kStart + 7ms);
  EXPECT_EQ(partial.timestamp, kStart + 7ms);
  EXPECT_FALSE(partial.has_telemetry);
  EXPECT_FALSE(partial.heartbeat);
  EXPECT_FALSE(partial.attitude);
  EXPECT_FALSE(partial.global_position);
  EXPECT_FALSE(partial.battery_status);
  EXPECT_EQ(controller.GetTime(), kStart + 7ms);
  EXPECT_EQ(controller.NextTelemetryTime(), kStart + 20ms);
  for (int tick = 1; tick <= 50; ++tick) {
    const auto telemetry = controller.Process(kStart + 1013ms);
    EXPECT_TRUE(telemetry.has_telemetry);
    EXPECT_EQ(telemetry.timestamp, kStart + tick * 20ms);
    EXPECT_EQ(controller.GetTime(), telemetry.timestamp);
    ASSERT_TRUE(telemetry.attitude);
    EXPECT_EQ(telemetry.attitude->timestamp, telemetry.timestamp);
    EXPECT_EQ(telemetry.global_position.has_value(), tick % 5 == 0);
    EXPECT_EQ(telemetry.battery_status.has_value(), tick % 25 == 0);
    EXPECT_EQ(telemetry.heartbeat.has_value(), tick == 50);
  }
  const auto remainder = controller.Process(kStart + 1013ms);
  EXPECT_EQ(remainder.timestamp, kStart + 1013ms);
  EXPECT_FALSE(remainder.has_telemetry);
  EXPECT_EQ(controller.GetTime(), kStart + 1013ms);
  EXPECT_FALSE(controller.Process(kStart + 1013ms).has_telemetry);
  EXPECT_DEBUG_DEATH(controller.Process(kStart + 1012ms), "");
  const auto telemetry = controller.Process(kStart + 1033ms);
  ASSERT_TRUE(telemetry.attitude);
  EXPECT_EQ(telemetry.attitude->timestamp, kStart + 1020ms);
  EXPECT_EQ(controller.GetTime(), kStart + 1020ms);
  const auto repeated = controller.Process(kStart + 1020ms);
  EXPECT_EQ(repeated.timestamp, kStart + 1020ms);
  EXPECT_FALSE(repeated.has_telemetry);
  EXPECT_FALSE(repeated.attitude);
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
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second,
                   1.0);
  EXPECT_NEAR(telemetry.attitude->orientation.pitch_radians, -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(
      telemetry.attitude->angular_velocity.pitch_radians_per_second, -1.0);
  telemetry = Steps(20);
  ASSERT_TRUE(telemetry.global_position);
  ASSERT_TRUE(telemetry.attitude);
  ASSERT_TRUE(telemetry.battery_status);
  EXPECT_EQ(telemetry.attitude->timestamp, kStart + 1500ms);
  EXPECT_DOUBLE_EQ(telemetry.global_position->position.x_meters, 0.125);
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second,
                   0.0);
  EXPECT_DOUBLE_EQ(telemetry.attitude->orientation.pitch_radians, -0.125);
  EXPECT_DOUBLE_EQ(
      telemetry.attitude->angular_velocity.pitch_radians_per_second, 0.0);
  const double charge = 2.0 - (1.0 + 10.0 * 0.125 + 0.375) / 3600.0;
  EXPECT_NEAR(telemetry.battery_status->charge_amp_hours, charge, 1e-11);
  EXPECT_NEAR(telemetry.battery_status->voltage_volts, 9.0 + 3.6 * charge / 2.0,
              1e-11);
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
  EXPECT_EQ(telemetry.heartbeat->timestamp, kStart + 2s);
  EXPECT_EQ(telemetry.heartbeat->state, State::kHolding);
  EXPECT_NEAR(telemetry.global_position->position.x_meters, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(telemetry.global_position->velocity.x_meters_per_second,
                   0.0);
  EXPECT_DOUBLE_EQ(telemetry.attitude->angular_velocity.roll_radians_per_second,
                   0.0);
  EXPECT_DOUBLE_EQ(telemetry.battery_status->current_amps, 1.0);
  EXPECT_NEAR(telemetry.battery_status->charge_amp_hours,
              2.0 - (1.5 + 10.0 * 0.5) / 3600.0, 1e-11);
}

TEST_F(ControllerTelemetryTest, HandlesZeroAndPartialStepReadinessTimeouts) {
  for (const auto timeout : {0ms, 7ms}) {
    ControllerStateMachine custom{model, logger, kStart, timeout};
    Controller local{model, battery, custom, kStart};
    local.Command(Command::kUnguard);
    const auto telemetry = local.Process(kStart + 20ms);
    EXPECT_EQ(custom.GetState(), State::kReady);
    ASSERT_TRUE(telemetry.attitude);
    EXPECT_EQ(telemetry.attitude->timestamp, kStart + 20ms);
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
  [[nodiscard]] double GetCurrentAmps() const { return current; }
  [[nodiscard]] Orientation GetOrientation() const { return {1.0, 2.0, 3.0}; }
  [[nodiscard]] AngularVelocity GetAngularVelocity() const {
    return {4.0, 5.0, 6.0};
  }
  [[nodiscard]] Position GetPosition() const { return {7.0, 8.0, 9.0}; }
  [[nodiscard]] LinearVelocity GetLinearVelocity() const {
    return {10.0, 11.0, 12.0};
  }
};

class RecordingBattery {
 public:
  std::vector<std::pair<double, simulation::Duration>> calls;
  bool Process(double current, simulation::Duration elapsed) {
    calls.emplace_back(current, elapsed);
    return true;
  }
  [[nodiscard]] double GetVoltageVolts() const { return 11.5; }
  [[nodiscard]] double GetChargeAmpHours() const { return 1.5; }
};

class StubStateMachine {
 public:
  using TimePoint = simulation::TimePoint;
  using CommandType = Controller<>::CommandType;

  StubStateMachine(StubModel& model, TimePoint start)
      : model_(model), start_(start) {}

  ~StubStateMachine() = default;
  StubStateMachine(const StubStateMachine&) = delete;
  StubStateMachine& operator=(const StubStateMachine&) = delete;
  StubStateMachine(StubStateMachine&&) = delete;
  StubStateMachine& operator=(StubStateMachine&&) = delete;

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
  [[nodiscard]] State GetState() const { return State::kHolding; }
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
  constexpr simulation::TimePoint kStart{123s};
  StubModel model;
  RecordingBattery battery;
  StubStateMachine state_machine{model, kStart};
  Controller<StubModel, RecordingBattery, StubStateMachine> controller{
      model, battery, state_machine, kStart};
  EXPECT_TRUE(state_machine.endpoints.empty());
  EXPECT_TRUE(battery.calls.empty());
  EXPECT_FALSE(controller.Command(Command::kToManual));
  EXPECT_EQ(state_machine.command, Command::kToManual);
  EXPECT_FALSE(controller.SetManualControl({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}));
  EXPECT_DOUBLE_EQ(state_machine.target.y_meters, 2.0);
  EXPECT_DOUBLE_EQ(state_machine.orientation.pitch_radians, 5.0);
  EXPECT_TRUE(controller.SetAutomaticControl({7.0, 8.0, 9.0}, {}));
  EXPECT_DOUBLE_EQ(state_machine.target.z_meters, 9.0);
  const auto telemetry = controller.Process(kStart + 20ms);
  ASSERT_EQ(state_machine.endpoints.size(), 2U);
  EXPECT_EQ(state_machine.endpoints[0], kStart + 20ms);
  EXPECT_EQ(state_machine.endpoints[1], kStart + 20ms);
  ASSERT_EQ(battery.calls.size(), 2U);
  EXPECT_DOUBLE_EQ(battery.calls[0].first, 7.0);
  EXPECT_EQ(battery.calls[0].second, 7ms);
  EXPECT_DOUBLE_EQ(battery.calls[1].first, 2.0);
  EXPECT_EQ(battery.calls[1].second, 13ms);
  ASSERT_TRUE(telemetry.attitude);
  EXPECT_EQ(telemetry.attitude->timestamp, kStart + 20ms);
  EXPECT_DOUBLE_EQ(telemetry.attitude->orientation.yaw_radians, 3.0);
  EXPECT_DOUBLE_EQ(telemetry.attitude->angular_velocity.yaw_radians_per_second,
                   6.0);
  Telemetry heartbeat_telemetry;
  for (int tick = 1; tick < 50; ++tick) {
    heartbeat_telemetry = controller.Process(controller.GetTime() + 20ms);
  }
  ASSERT_TRUE(heartbeat_telemetry.heartbeat);
  EXPECT_EQ(heartbeat_telemetry.heartbeat->timestamp, kStart + 1s);
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
