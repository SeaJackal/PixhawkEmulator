#include "controller_state_machine.h"

#include <gtest/gtest-death-test.h>
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <string>

#include "dynamic_model.h"
#include "logger.h"
#include "simulation_time.h"

namespace {

using Controller = ControllerStateMachine;
using State = Controller::State;
using Command = Controller::CommandType;
using namespace std::chrono_literals;

class ControllerTest : public ::testing::Test {
 protected:
  static constexpr Controller::TimePoint kStart{42s};
  DynamicModel model;
  Logger logger{kStart};
  Controller controller{model, logger, kStart};

  void Ready();

  void Manual();

  void Automatic();
};

// -----------------------------------------------------------------------------

void ControllerTest::Ready() {
  ASSERT_TRUE(controller.Command(Command::kUnguard));
  ASSERT_EQ(controller.Process(kStart + 1s), kStart + 1s);
  ASSERT_EQ(controller.GetState(), State::kReady);
}

void ControllerTest::Manual() {
  Ready();
  ASSERT_TRUE(controller.Command(Command::kToManual));
}

void ControllerTest::Automatic() {
  Ready();
  ASSERT_TRUE(controller.Command(Command::kToAutomatic));
}

TEST_F(ControllerTest, StartsGuardedAndAdvancesIdleTime) {
  EXPECT_EQ(controller.GetState(), State::kGuarded);
  EXPECT_EQ(controller.Process(kStart), kStart);
  EXPECT_EQ(controller.Process(kStart + 10s), kStart + 10s);
  EXPECT_EQ(model.GetPosition().x_meters, 0.0);
}

TEST_F(ControllerTest, ReadinessInterruptsOnceAndLogsRelativeTime) {
  ASSERT_TRUE(controller.Command(Command::kUnguard));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1s);
  EXPECT_EQ(controller.GetState(), State::kReady);
  EXPECT_NE(logger.GetText().find(
                "[1.000000000] [DEBUG] controller: not ready -> ready "
                "(readiness timeout)"),
            std::string::npos);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
}

TEST_F(ControllerTest, CustomAndZeroReadinessTimeouts) {
  Controller custom{model, logger, kStart, 250ms};
  ASSERT_TRUE(custom.Command(Command::kUnguard));
  EXPECT_EQ(custom.Process(kStart + 249ms), kStart + 249ms);
  EXPECT_EQ(custom.GetState(), State::kNotReady);
  EXPECT_EQ(custom.Process(kStart + 250ms), kStart + 250ms);
  EXPECT_EQ(custom.GetState(), State::kReady);

  Controller immediate{model, logger, kStart, 0ms};
  ASSERT_TRUE(immediate.Command(Command::kUnguard));
  EXPECT_EQ(immediate.Process(kStart + 1s), kStart);
  EXPECT_EQ(immediate.GetState(), State::kReady);
  EXPECT_EQ(immediate.Process(kStart + 1s), kStart + 1s);
}

TEST_F(ControllerTest, ValidatesEveryCommandInEveryState) {
  const std::array states{State::kGuarded, State::kNotReady,  State::kReady,
                          State::kManual,  State::kAutomatic, State::kHolding};
  const std::array commands{Command::kUnguard, Command::kToManual,
                            Command::kToAutomatic, Command::kHold,
                            Command::kLand};
  const std::array<std::array<bool, 5>, 6> accepted = {
      {{true, false, false, false, false},
       {false, false, false, false, false},
       {false, true, true, false, false},
       {false, false, true, true, true},
       {false, true, false, true, true},
       {false, true, true, false, true}}};
  const std::array destinations{State::kNotReady, State::kManual,
                                State::kAutomatic, State::kHolding,
                                State::kReady};

  for (std::size_t i = 0; i < states.size(); ++i) {
    for (std::size_t j = 0; j < commands.size(); ++j) {
      SCOPED_TRACE(std::to_string(i) + ":" + std::to_string(j));
      DynamicModel local_model;
      Logger local_logger{kStart};
      Controller local{local_model, local_logger, kStart};
      if (states[i] != State::kGuarded) {
        ASSERT_TRUE(local.Command(Command::kUnguard));
      }
      if (i >= 2) {
        ASSERT_EQ(local.Process(kStart + 1s), kStart + 1s);
      }
      if (states[i] == State::kManual || states[i] == State::kHolding) {
        ASSERT_TRUE(local.Command(Command::kToManual));
      } else if (states[i] == State::kAutomatic) {
        ASSERT_TRUE(local.Command(Command::kToAutomatic));
      }
      if (states[i] == State::kHolding) {
        ASSERT_TRUE(local.Command(Command::kHold));
      }
      ASSERT_EQ(local.GetState(), states[i]);
      local_logger.Clear();
      EXPECT_EQ(local.Command(commands[j]), accepted[i][j]);
      const std::string expected_level =
          accepted[i][j] ? "[DEBUG]" : "[WARNING]";
      EXPECT_NE(local_logger.GetText().find(expected_level), std::string::npos);
      EXPECT_EQ(local.GetState(), accepted[i][j] ? destinations[j] : states[i]);
    }
  }
}

TEST_F(ControllerTest, RejectsWrongModeInputsAndReplacesAcceptedTarget) {
  EXPECT_FALSE(controller.SetManualControl({10.0, 0.0, 0.0}, {}));
  EXPECT_FALSE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  Manual();
  EXPECT_FALSE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  ASSERT_TRUE(controller.SetManualControl({10.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  ASSERT_TRUE(controller.SetManualControl({-10.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 3s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_NE(logger.GetText().find("[WARNING] controller: rejected"),
            std::string::npos);
}

TEST_F(ControllerTest, ArrivalKeepsManualModeAndInterruptsOnlyOnce) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {0.0, -2.0, 0.5}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 3s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 1.0);
  EXPECT_DOUBLE_EQ(model.GetOrientation().pitch_radians, -2.0);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
}

TEST_F(ControllerTest, ArrivalKeepsAutomaticModeAndOriginalTimeout) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({0.25, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 1250ms), kStart + 1250ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  EXPECT_EQ(controller.Process(kStart + 1400ms), kStart + 1400ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
}

TEST_F(ControllerTest, SatisfiedTargetDoesNotInterruptProcessing) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
}

TEST_F(ControllerTest, ModeWithoutTargetDoesNotProduceArrival) {
  Manual();
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
  EXPECT_EQ(controller.GetState(), State::kManual);
}

TEST_F(ControllerTest, SatisfiedAutomaticTargetStopsOnlyAtTimeout) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({}, {}));
  EXPECT_EQ(controller.Process(kStart + 1250ms), kStart + 1250ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  ASSERT_TRUE(controller.SetAutomaticControl({}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1750ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
}

TEST_F(ControllerTest, ManualAcceptsNextTargetAfterArrivalWithoutModeCommand) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 2s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  ASSERT_TRUE(controller.SetManualControl({2.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 2.0);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
}

TEST_F(ControllerTest, AutomaticAcceptsNextTargetAfterArrivalAndResetsTimeout) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({0.25, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1250ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  ASSERT_TRUE(controller.SetAutomaticControl({0.5, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1750ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
}

TEST_F(ControllerTest, RepeatedManualCommandLogsWarningAndPreservesTarget) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({2.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  logger.Clear();
  EXPECT_FALSE(controller.Command(Command::kToManual));
  EXPECT_EQ(
      logger.GetText(),
      "[2.000000000] [WARNING] controller: rejected to manual in manual\n");
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 2.0);
  EXPECT_EQ(controller.GetState(), State::kManual);
}

TEST_F(ControllerTest, AutomaticTimesOutWithoutInputAtExactly500Milliseconds) {
  Automatic();
  EXPECT_EQ(controller.Process(kStart + 1499ms), kStart + 1499ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  EXPECT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
  EXPECT_NE(logger.GetText().find(
                "[1.500000000] [DEBUG] controller: automatic -> holding "
                "(automatic input timeout)"),
            std::string::npos);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
}

TEST_F(ControllerTest, AutomaticTimeoutStopsBothTranslationAndRotation) {
  Automatic();
  ASSERT_TRUE(
      controller.SetAutomaticControl({10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}));
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
  EXPECT_DOUBLE_EQ(model.GetOrientation().roll_radians, 0.5);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetAngularVelocity().roll_radians_per_second, 0.0);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
}

TEST_F(ControllerTest,
       EachAutomaticInputIncludingUnchangedTargetResetsTimeout) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 1400ms), kStart + 1400ms);
  ASSERT_TRUE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kAutomatic);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 1900ms);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.9, 1e-12);
}

TEST_F(ControllerTest, RepeatedModeAndRejectedInputsDoNotResetTimeout) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 1400ms), kStart + 1400ms);
  EXPECT_FALSE(controller.Command(Command::kToAutomatic));
  EXPECT_FALSE(controller.Command(Command::kUnguard));
  EXPECT_FALSE(controller.SetManualControl({20.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 1500ms);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
}

TEST_F(ControllerTest,
       ModeSwitchCancelsTargetAndTimeoutAndReentryRestartsTimer) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({10.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 1250ms), kStart + 1250ms);
  ASSERT_TRUE(controller.Command(Command::kToManual));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.25);
  ASSERT_TRUE(controller.Command(Command::kToAutomatic));
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 2500ms);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.25);
}

TEST_F(ControllerTest, SimultaneousArrivalAndTimeoutProduceOneHoldingEvent) {
  Automatic();
  ASSERT_TRUE(controller.SetAutomaticControl({0.5, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1500ms);
  EXPECT_EQ(controller.GetState(), State::kHolding);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
  EXPECT_NE(
      logger.GetText().find(
          "[DEBUG] controller: automatic -> holding (automatic input timeout)"),
      std::string::npos);
  logger.Clear();
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_TRUE(logger.GetText().empty());
}

TEST_F(ControllerTest, HoldFreezesCompletePoseAndLandRejectsAirbornePosition) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({0.0, 0.0, 10.0}, {1.0, 2.0, 3.0}));
  ASSERT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_FALSE(controller.Command(Command::kLand));
  ASSERT_TRUE(controller.Command(Command::kHold));
  EXPECT_FALSE(controller.Command(Command::kLand));
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
  EXPECT_DOUBLE_EQ(model.GetPosition().z_meters, 0.5);
  EXPECT_DOUBLE_EQ(model.GetOrientation().roll_radians, 0.5);
  EXPECT_DOUBLE_EQ(model.GetOrientation().pitch_radians, 0.5);
  EXPECT_DOUBLE_EQ(model.GetOrientation().yaw_radians, 0.5);
}

TEST_F(ControllerTest, LandAtGroundStopsMotionAndEntersReady) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({10.0, 0.0, 0.0}, {1.0, 2.0, 3.0}));
  ASSERT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_TRUE(controller.Command(Command::kLand));
  EXPECT_EQ(controller.GetState(), State::kReady);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.5);
  EXPECT_DOUBLE_EQ(model.GetOrientation().yaw_radians, 0.5);
}

TEST_F(ControllerTest, UnreachableTargetDoesNotGenerateFalseArrival) {
  DynamicModel stationary{0.0, 0.0};
  Controller local{stationary, logger, kStart, 0ms};
  ASSERT_TRUE(local.Command(Command::kUnguard));
  ASSERT_EQ(local.Process(kStart), kStart);
  ASSERT_TRUE(local.Command(Command::kToManual));
  ASSERT_TRUE(local.SetManualControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(local.Process(kStart + 10s), kStart + 10s);
  EXPECT_EQ(local.GetState(), State::kManual);
  ASSERT_TRUE(local.Command(Command::kToAutomatic));
  ASSERT_TRUE(local.SetAutomaticControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(local.Process(kStart + 11s), kStart + 10500ms);
  EXPECT_EQ(local.GetState(), State::kHolding);
}

TEST_F(ControllerTest, ArrivalRoundsUpToClockResolution) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0 / 3.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s),
            kStart + 1s +
                std::chrono::ceil<simulation::Duration>(
                    std::chrono::duration<double>{1.0 / 3.0}));
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 1.0 / 3.0);
}

TEST_F(ControllerTest, EqualTimeDoesNotMovePendingTarget) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 1s), kStart + 1s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 0.0);
  EXPECT_EQ(controller.Process(kStart + 1s), kStart + 1s);
  EXPECT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 2s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 1.0);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
}

TEST_F(ControllerTest, ExactEndpointConsumesArrivalOnlyOnce) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_EQ(controller.GetState(), State::kManual);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 1.0);
  EXPECT_DOUBLE_EQ(model.GetOrientation().yaw_radians, 1.0);
}

TEST_F(ControllerTest, RepeatedSatisfiedTargetsDoNotInterruptProcessing) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({}, {}));
  EXPECT_EQ(controller.Process(kStart + 1s), kStart + 1s);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  ASSERT_TRUE(controller.SetManualControl({}, {}));
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
  EXPECT_EQ(controller.GetState(), State::kManual);
}

TEST_F(ControllerTest, ReplacingTargetStopsOnlyAtNewTarget) {
  Manual();
  ASSERT_TRUE(controller.SetManualControl({1.0, 0.0, 0.0}, {}));
  ASSERT_EQ(controller.Process(kStart + 1500ms), kStart + 1500ms);
  ASSERT_TRUE(controller.SetManualControl({2.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2500ms), kStart + 2500ms);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 3s);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, 2.0);
  EXPECT_EQ(controller.Process(kStart + 5s), kStart + 5s);
}

TEST_F(ControllerTest, RejectedAutomaticInputDoesNotChangeReadinessDeadline) {
  ASSERT_TRUE(controller.Command(Command::kUnguard));
  ASSERT_EQ(controller.Process(kStart + 100ms), kStart + 100ms);
  EXPECT_FALSE(controller.SetAutomaticControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 750ms), kStart + 750ms);
  EXPECT_EQ(controller.GetState(), State::kNotReady);
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 1s);
  EXPECT_EQ(controller.GetState(), State::kReady);
}

TEST_F(ControllerTest, RejectedAutomaticInputDoesNotLimitIdleProcessing) {
  EXPECT_FALSE(controller.SetAutomaticControl({1.0, 0.0, 0.0}, {}));
  EXPECT_EQ(controller.Process(kStart + 2s), kStart + 2s);
  EXPECT_EQ(controller.Process(kStart + 3s), kStart + 3s);
  EXPECT_EQ(controller.GetState(), State::kGuarded);
}

#ifndef NDEBUG
TEST_F(ControllerTest, RejectsBackwardsTime) {
  EXPECT_DEATH(controller.Process(kStart - 1s), "until >= time_");
}

TEST_F(ControllerTest, RejectsNegativeReadinessTimeout) {
  EXPECT_DEATH((Controller{model, logger, kStart, -1ms}), "readiness_timeout_");
}
#endif

}
