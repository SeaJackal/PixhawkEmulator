#pragma once

#include "dynamic_model.h"
#include "logger.h"
#include "simulation_time.h"

#include <cstdint>
#include <optional>
#include <string_view>

class ControllerStateMachine {
 public:
  using TimePoint = simulation::TimePoint;
  using Duration = simulation::Duration;

  enum class State { kGuarded, kNotReady, kReady, kManual, kAutomatic, kHolding };
  enum class CommandType : std::uint8_t {
    kUnguard = 0,
    kToManual = 1,
    kToAutomatic = 2,
    kHold = 3,
    kLand = 4
  };

  static constexpr Duration kAutomaticTimeout = std::chrono::milliseconds{500};

  ControllerStateMachine(DynamicModel& model, Logger& logger,
                         TimePoint start_time,
                         Duration readiness_timeout = std::chrono::seconds{1});

  bool SetAutomaticControl(const Position& position,
                           const Orientation& orientation);
  bool SetManualControl(const Position& position,
                        const Orientation& orientation);
  bool Command(CommandType command);
  TimePoint Process(TimePoint until);

  [[nodiscard]] State GetState() const;

 private:
  bool SetControl(State mode, const Position& position,
                  const Orientation& orientation);
  void ChangeState(State state, std::string_view reason);

  DynamicModel& model_;
  Logger& logger_;
  TimePoint time_;
  Duration readiness_timeout_;
  State state_ = State::kGuarded;
  std::optional<TimePoint> deadline_;
};
