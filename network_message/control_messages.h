#pragma once

#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "simulation_time.h"

struct CommandMessage {
  simulation::TimePoint timestamp;
  ControllerStateMachine::CommandType command;
};

struct AutomaticControlMessage {
  simulation::TimePoint timestamp;
  Position position;
  Orientation orientation;
};

struct ManualControlMessage {
  simulation::TimePoint timestamp;
  Position position;
  Orientation orientation;
};
