#pragma once

#include "simulation_time.h"

class BatteryModel {
 public:
  static constexpr double kEmptyVoltageVolts = 9.0;
  static constexpr double kFullVoltageVolts = 12.6;
  static constexpr double kLowChargePercent = 20.0;
  static constexpr double kCriticalChargePercent = 10.0;

  enum class Status { kOk, kLow, kCritical };

  BatteryModel(double max_capacity_amp_hours, double start_capacity_amp_hours);

  bool Process(double current_amps, simulation::Duration elapsed);
  void Reset();

  [[nodiscard]] double GetChargeAmpHours() const;
  [[nodiscard]] double GetVoltageVolts() const;
  [[nodiscard]] Status GetStatus() const;

 private:
  double max_capacity_amp_hours_;
  double charge_amp_hours_;
};
