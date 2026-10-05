#pragma once

#include <algorithm>
#include <cassert>

#include "battery_model.h"
#include "controller_state_machine.h"
#include "telemetry.h"

template <typename Model = DynamicModel, typename Battery = BatteryModel,
          typename StateMachine = ControllerStateMachine>
class Controller {
 public:
  using TimePoint = simulation::TimePoint;
  using Duration = simulation::Duration;
  using State = ControllerStateMachine::State;
  using CommandType = ControllerStateMachine::CommandType;

  static constexpr Duration kHeartbeatPeriod = std::chrono::seconds{1};
  static constexpr Duration kAttitudePeriod = std::chrono::milliseconds{20};
  static constexpr Duration kGlobalPositionPeriod =
      std::chrono::milliseconds{100};
  static constexpr Duration kBatteryStatusPeriod =
      std::chrono::milliseconds{500};
  static constexpr Duration kProcessPeriod =
      std::min({kHeartbeatPeriod, kAttitudePeriod, kGlobalPositionPeriod,
                kBatteryStatusPeriod});

  Controller(Model& model, Battery& battery,
             StateMachine& state_machine, TimePoint start_time);
  ~Controller() = default;

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = delete;
  Controller& operator=(Controller&&) = delete;

  bool Command(CommandType command);
  bool SetManualControl(const Position& position,
                        const Orientation& orientation);
  bool SetAutomaticControl(const Position& position,
                           const Orientation& orientation);
  Telemetry Process();

 private:
  Telemetry CollectTelemetry();

  Model& model_;
  Battery& battery_;
  StateMachine& state_machine_;
  TimePoint time_;
  TimePoint next_heartbeat_;
  TimePoint next_attitude_;
  TimePoint next_global_position_;
  TimePoint next_battery_status_;
};

template <typename Model, typename Battery, typename StateMachine>
Controller<Model, Battery, StateMachine>::Controller(Model& model, Battery& battery,
                       StateMachine& state_machine,
                       TimePoint start_time)
    : model_(model),
      battery_(battery),
      state_machine_(state_machine),
      time_(start_time),
      next_heartbeat_(start_time + kHeartbeatPeriod),
      next_attitude_(start_time + kAttitudePeriod),
      next_global_position_(start_time + kGlobalPositionPeriod),
      next_battery_status_(start_time + kBatteryStatusPeriod) {}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::Command(CommandType command) {
  return state_machine_.Command(command);
}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::SetManualControl(const Position& position,
                                  const Orientation& orientation) {
  return state_machine_.SetManualControl(position, orientation);
}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::SetAutomaticControl(const Position& position,
                                     const Orientation& orientation) {
  return state_machine_.SetAutomaticControl(position, orientation);
}

template <typename Model, typename Battery, typename StateMachine>
Telemetry Controller<Model, Battery, StateMachine>::Process() {
  const auto until = time_ + kProcessPeriod;
  while (time_ < until) {
    const double current_amps = model_.GetCurrentAmps();
    const auto reached = state_machine_.Process(until);
    assert(reached >= time_ && reached <= until);
    battery_.Process(current_amps, reached - time_);
    time_ = reached;
  }
  return CollectTelemetry();
}

template <typename Model, typename Battery, typename StateMachine>
Telemetry Controller<Model, Battery, StateMachine>::CollectTelemetry() {
  Telemetry telemetry;
  if (time_ == next_heartbeat_) {
    const auto state = state_machine_.GetState();
    const bool ready = state != State::kGuarded && state != State::kNotReady;
    telemetry.heartbeat = HeartbeatMessage{time_, state, ready};
    next_heartbeat_ += kHeartbeatPeriod;
  }
  if (time_ == next_attitude_) {
    telemetry.attitude = AttitudeMessage{time_, model_.GetOrientation(),
                                         model_.GetAngularVelocity()};
    next_attitude_ += kAttitudePeriod;
  }
  if (time_ == next_global_position_) {
    telemetry.global_position = GlobalPositionMessage{
        time_, model_.GetPosition(), model_.GetLinearVelocity()};
    next_global_position_ += kGlobalPositionPeriod;
  }
  if (time_ == next_battery_status_) {
    telemetry.battery_status = BatteryStatusMessage{
        time_, battery_.GetVoltageVolts(), model_.GetCurrentAmps(),
        battery_.GetChargeAmpHours()};
    next_battery_status_ += kBatteryStatusPeriod;
  }
  return telemetry;
}
