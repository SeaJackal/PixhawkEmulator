#pragma once

#include "simulation_time.h"

struct Position {
  double x_meters = 0.0;
  double y_meters = 0.0;
  double z_meters = 0.0;
};

struct Orientation {
  double roll_radians = 0.0;
  double pitch_radians = 0.0;
  double yaw_radians = 0.0;
};

struct LinearVelocity {
  double x_meters_per_second = 0.0;
  double y_meters_per_second = 0.0;
  double z_meters_per_second = 0.0;
};

struct AngularVelocity {
  double roll_radians_per_second = 0.0;
  double pitch_radians_per_second = 0.0;
  double yaw_radians_per_second = 0.0;
};

class DynamicModel {
 public:
  static constexpr double kIdleCurrentAmps = 1.0;
  static constexpr double kMovingCurrentAmps = 10.0;

  explicit DynamicModel(double linear_speed_meters_per_second = 1.0,
               double angular_speed_radians_per_second = 1.0);

  void Command(const Position& position, const Orientation& orientation);
  simulation::Duration Process(simulation::Duration elapsed);

  [[nodiscard]] Position GetPosition() const;
  [[nodiscard]] Orientation GetOrientation() const;
  [[nodiscard]] LinearVelocity GetLinearVelocity() const;
  [[nodiscard]] AngularVelocity GetAngularVelocity() const;
  [[nodiscard]] double GetCurrentAmps() const;

 private:
  double linear_speed_meters_per_second_;
  double angular_speed_radians_per_second_;
  Position position_;
  Orientation orientation_;
  Position target_position_;
  Orientation target_orientation_;
};
