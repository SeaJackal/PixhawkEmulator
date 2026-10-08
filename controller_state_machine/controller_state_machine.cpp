#include "controller_state_machine.h"

#include <algorithm>
#include <cassert>
#include <string>

#include "dynamic_model.h"
#include "logger.h"

namespace {

const char* StateName(ControllerStateMachine::State state) {
  using State = ControllerStateMachine::State;
  switch (state) {
    case State::kGuarded:
      return "guarded";
    case State::kNotReady:
      return "not ready";
    case State::kReady:
      return "ready";
    case State::kManual:
      return "manual";
    case State::kAutomatic:
      return "automatic";
    case State::kHolding:
      return "holding";
  }
  return "unknown";
}

const char* CommandName(ControllerStateMachine::CommandType command) {
  using Command = ControllerStateMachine::CommandType;
  switch (command) {
    case Command::kUnguard:
      return "unguard";
    case Command::kToManual:
      return "to manual";
    case Command::kToAutomatic:
      return "to automatic";
    case Command::kHold:
      return "hold";
    case Command::kLand:
      return "land";
  }
  return "unknown";
}

}  // namespace

ControllerStateMachine::ControllerStateMachine(DynamicModel& model,
                                               Logger& logger,
                                               TimePoint start_time,
                                               Duration readiness_timeout)
    : model_(model),
      logger_(logger),
      time_(start_time),
      readiness_timeout_(readiness_timeout) {
  model_.Command(model_.GetPosition(), model_.GetOrientation());
  assert(readiness_timeout_ >= Duration::zero());
}

bool ControllerStateMachine::SetAutomaticControl(
    const Position& position, const Orientation& orientation) {
  return SetControl(State::kAutomatic, position, orientation);
}

bool ControllerStateMachine::SetManualControl(const Position& position,
                                              const Orientation& orientation) {
  return SetControl(State::kManual, position, orientation);
}

bool ControllerStateMachine::SetControl(State mode, const Position& position,
                                        const Orientation& orientation) {
  if (state_ != mode) {
    logger_.Log(Logger::Level::kWarning, time_,
                std::string("controller: rejected ") + StateName(mode) +
                    " target in " + StateName(state_));
    return false;
  }
  model_.Command(position, orientation);
  if (state_ == State::kAutomatic) {
    deadline_ = time_ + kAutomaticTimeout;
  }
  logger_.Log(
      Logger::Level::kDebug, time_,
      std::string("controller: accepted ") + StateName(mode) + " target");
  return true;
}

bool ControllerStateMachine::Command(CommandType command) {
  switch (command) {
    case CommandType::kUnguard:
      if (state_ == State::kGuarded) {
        ChangeState(State::kNotReady, "unguard");
        return true;
      }
      break;
    case CommandType::kToManual:
    case CommandType::kToAutomatic: {
      const State mode = command == CommandType::kToManual ? State::kManual
                                                           : State::kAutomatic;
      if (state_ == mode) {
        break;
      }
      if (state_ == State::kReady || state_ == State::kManual ||
          state_ == State::kAutomatic || state_ == State::kHolding) {
        ChangeState(mode, CommandName(command));
        return true;
      }
      break;
    }
    case CommandType::kHold:
      if (state_ == State::kManual || state_ == State::kAutomatic) {
        ChangeState(State::kHolding, "hold");
        return true;
      }
      break;
    case CommandType::kLand:
      if ((state_ == State::kManual || state_ == State::kAutomatic ||
           state_ == State::kHolding) &&
          model_.GetPosition().z_meters == 0.0) {
        ChangeState(State::kReady, "land");
        return true;
      }
      break;
  }
  logger_.Log(Logger::Level::kWarning, time_,
              std::string("controller: rejected ") + CommandName(command) +
                  " in " + StateName(state_));
  return false;
}

ControllerStateMachine::TimePoint ControllerStateMachine::Process(
    TimePoint until) {
  assert(until >= time_);
  const TimePoint endpoint = deadline_ ? std::min(until, *deadline_) : until;

  const auto requested = endpoint - time_;
  const auto processed = model_.Process(requested);
  if (processed < requested) {
    time_ += processed;
  } else {
    time_ = endpoint;
  }

  if (deadline_ && time_ == *deadline_) {
    if (state_ == State::kNotReady) {
      ChangeState(State::kReady, "readiness timeout");
    } else if (state_ == State::kAutomatic) {
      ChangeState(State::kHolding, "automatic input timeout");
    }
  }
  return time_;
}

void ControllerStateMachine::ChangeState(State state, std::string_view reason) {
  const State previous = state_;
  model_.Command(model_.GetPosition(), model_.GetOrientation());
  deadline_.reset();
  state_ = state;
  if (state_ == State::kNotReady) {
    deadline_ = time_ + readiness_timeout_;
  } else if (state_ == State::kAutomatic) {
    deadline_ = time_ + kAutomaticTimeout;
  }
  logger_.Log(Logger::Level::kDebug, time_,
              std::string("controller: ") + StateName(previous) + " -> " +
                  StateName(state_) + " (" + std::string(reason) + ")");
}

ControllerStateMachine::State ControllerStateMachine::GetState() const {
  return state_;
}
