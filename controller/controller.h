#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <vector>

#include "battery_model.h"
#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "simulation_time.h"
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

  Controller(Model& model, Battery& battery, StateMachine& state_machine,
             TimePoint start_time);
  ~Controller() = default;

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = delete;
  Controller& operator=(Controller&&) = delete;

  bool Command(CommandType command);
  void AddGnssLoss(TimePoint start, Duration duration);
  bool SetManualControl(const Position& position,
                        const Orientation& orientation);
  bool SetAutomaticControl(const Position& position,
                           const Orientation& orientation);
  Telemetry Process(TimePoint until);
  [[nodiscard]] TimePoint GetTime() const { return time_; }
  [[nodiscard]] TimePoint NextTelemetryTime() const {
    return std::min({next_heartbeat_, next_attitude_, next_global_position_,
                     next_battery_status_});
  }

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
  struct GnssLoss {
    TimePoint start;
    TimePoint end;
  };
  std::vector<GnssLoss> gnss_losses_;
};

template <typename Model, typename Battery, typename StateMachine>
Controller<Model, Battery, StateMachine>::Controller(
    Model& model, Battery& battery, StateMachine& state_machine,
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
void Controller<Model, Battery, StateMachine>::AddGnssLoss(TimePoint start,
                                                           Duration duration) {
  if (duration < Duration::zero() || start > TimePoint::max() - duration) {
    throw std::invalid_argument(
        "Invalid GNSS loss duration or timestamp overflow");
  }
  if (duration == Duration::zero()) {
    return;
  }
  gnss_losses_.push_back({start, start + duration});
}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::Command(CommandType command) {
  return state_machine_.Command(command);
}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::SetManualControl(
    const Position& position, const Orientation& orientation) {
  return state_machine_.SetManualControl(position, orientation);
}

template <typename Model, typename Battery, typename StateMachine>
bool Controller<Model, Battery, StateMachine>::SetAutomaticControl(
    const Position& position, const Orientation& orientation) {
  return state_machine_.SetAutomaticControl(position, orientation);
}

template <typename Model, typename Battery, typename StateMachine>
Telemetry Controller<Model, Battery, StateMachine>::Process(TimePoint until) {
  assert(until >= time_);
  const auto endpoint = std::min(until, NextTelemetryTime());
  while (time_ < endpoint) {
    const double current_amps = model_.GetCurrentAmps();
    const auto reached = state_machine_.Process(endpoint);
    assert(reached >= time_ && reached <= endpoint);
    battery_.Process(current_amps, reached - time_);
    time_ = reached;
  }
  return CollectTelemetry();
}

template <typename Model, typename Battery, typename StateMachine>
Telemetry Controller<Model, Battery, StateMachine>::CollectTelemetry() {
  Telemetry telemetry;
  telemetry.timestamp = time_;
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
    const bool gnss_valid = std::none_of(
        gnss_losses_.begin(), gnss_losses_.end(), [this](const GnssLoss& loss) {
          return loss.start <= time_ && time_ < loss.end;
        });
    telemetry.global_position = GlobalPositionMessage{
        time_, model_.GetPosition(), model_.GetLinearVelocity(), gnss_valid};
    next_global_position_ += kGlobalPositionPeriod;
  }
  if (time_ == next_battery_status_) {
    telemetry.battery_status = BatteryStatusMessage{
        time_, battery_.GetVoltageVolts(), model_.GetCurrentAmps(),
        battery_.GetChargeAmpHours()};
    next_battery_status_ += kBatteryStatusPeriod;
  }
  telemetry.has_telemetry = telemetry.heartbeat.has_value() ||
                            telemetry.attitude.has_value() ||
                            telemetry.global_position.has_value() ||
                            telemetry.battery_status.has_value();
  return telemetry;
}
