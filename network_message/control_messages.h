#pragma once

#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "simulation_time.h"

struct CommandMessage {
  simulation::TimePoint timestamp;
  ControllerStateMachine::CommandType command =
      ControllerStateMachine::CommandType::kUnguard;
};

struct ControlMessage {
  simulation::TimePoint timestamp;
  Position position;
  Orientation orientation;
};
