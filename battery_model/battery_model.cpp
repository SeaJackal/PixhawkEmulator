#include "battery_model.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

bool BatteryModel::Process(double current_amps,
                           simulation::Duration elapsed) {
  if (charge_amp_hours_ == 0.0) {
    throw std::runtime_error("Battery is empty");
  }

  const double seconds = std::chrono::duration<double>(elapsed).count();
  assert(std::isfinite(current_amps));
  assert(elapsed >= simulation::Duration::zero());

  const double critical_charge_amp_hours =
      kCapacityAmpHours * (kCriticalChargePercent / 100.0);
  const double consumed_amp_hours = current_amps * (seconds / 3600.0);
  charge_amp_hours_ = std::clamp(charge_amp_hours_ - consumed_amp_hours,
                               critical_charge_amp_hours, kCapacityAmpHours);
  return GetStatus() != Status::kCritical;
}

void BatteryModel::Reset() { charge_amp_hours_ = kCapacityAmpHours; }

double BatteryModel::GetChargeAmpHours() const { return charge_amp_hours_; }

double BatteryModel::GetVoltageVolts() const {
  const double fraction = charge_amp_hours_ / kCapacityAmpHours;
  return kEmptyVoltageVolts +
         fraction * (kFullVoltageVolts - kEmptyVoltageVolts);
}

BatteryModel::Status BatteryModel::GetStatus() const {
  const double charge_percent = 100.0 * (charge_amp_hours_ / kCapacityAmpHours);
  if (charge_percent <= kCriticalChargePercent) {
    return Status::kCritical;
  }
  if (charge_percent <= kLowChargePercent) {
    return Status::kLow;
  }
  return Status::kOk;
}
