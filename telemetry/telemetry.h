#pragma once

#include "controller_state_machine.h"
#include "dynamic_model.h"
#include "simulation_time.h"

#include <optional>

struct HeartbeatMessage {
  simulation::TimePoint timestamp;
  ControllerStateMachine::State state;
  bool ready;
};

struct AttitudeMessage {
  simulation::TimePoint timestamp;
  Orientation orientation;
  AngularVelocity angular_velocity;
};

struct GlobalPositionMessage {
  simulation::TimePoint timestamp;
  Position position;
  LinearVelocity velocity;
};

struct BatteryStatusMessage {
  simulation::TimePoint timestamp;
  double voltage_volts;
  double current_amps;
  double charge_amp_hours;
};

struct Telemetry {
  std::optional<HeartbeatMessage> heartbeat;
  std::optional<AttitudeMessage> attitude;
  std::optional<GlobalPositionMessage> global_position;
  std::optional<BatteryStatusMessage> battery_status;
};
