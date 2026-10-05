#include "dynamic_model.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace {

double AdvanceAngle(double current, double target, double step) {
  const double difference = target - current;
  if (std::abs(difference) <= step) {
    return target;
  }
  return current + std::copysign(step, difference);
}

}  // namespace

DynamicModel::DynamicModel(double linear_speed_meters_per_second,
                           double angular_speed_radians_per_second)
    : linear_speed_meters_per_second_(linear_speed_meters_per_second),
      angular_speed_radians_per_second_(angular_speed_radians_per_second) {
  assert(std::isfinite(linear_speed_meters_per_second_));
  assert(linear_speed_meters_per_second_ >= 0.0);
  assert(std::isfinite(angular_speed_radians_per_second_));
  assert(angular_speed_radians_per_second_ >= 0.0);
}

void DynamicModel::Command(const Position& position,
                           const Orientation& orientation) {
  assert(std::isfinite(position.x_meters));
  assert(std::isfinite(position.y_meters));
  assert(std::isfinite(position.z_meters));
  assert(std::isfinite(orientation.roll_radians));
  assert(std::isfinite(orientation.pitch_radians));
  assert(std::isfinite(orientation.yaw_radians));
  target_position_ = position;
  target_orientation_ = orientation;
}

simulation::Duration DynamicModel::Process(simulation::Duration elapsed) {
  assert(elapsed >= simulation::Duration::zero());
  const double seconds = std::chrono::duration<double>(elapsed).count();

  if (position_.x_meters == target_position_.x_meters &&
      position_.y_meters == target_position_.y_meters &&
      position_.z_meters == target_position_.z_meters &&
      orientation_.roll_radians == target_orientation_.roll_radians &&
      orientation_.pitch_radians == target_orientation_.pitch_radians &&
      orientation_.yaw_radians == target_orientation_.yaw_radians) {
    return elapsed;
  }

  const double dx = target_position_.x_meters - position_.x_meters;
  const double dy = target_position_.y_meters - position_.y_meters;
  const double dz = target_position_.z_meters - position_.z_meters;
  const double distance = std::hypot(dx, dy, dz);
  const auto travel_time = [](double distance, double speed) {
    if (distance == 0.0) {
      return 0.0;
    }
    return speed == 0.0 ? std::numeric_limits<double>::infinity()
                        : distance / speed;
  };
  const double target_seconds = std::max(
      {travel_time(distance, linear_speed_meters_per_second_),
       travel_time(std::abs(target_orientation_.roll_radians -
                            orientation_.roll_radians),
                   angular_speed_radians_per_second_),
       travel_time(std::abs(target_orientation_.pitch_radians -
                            orientation_.pitch_radians),
                   angular_speed_radians_per_second_),
       travel_time(std::abs(target_orientation_.yaw_radians -
                            orientation_.yaw_radians),
                   angular_speed_radians_per_second_)});
  const std::chrono::duration<long double> target_time{target_seconds};
  if (target_time <= elapsed) {
    position_ = target_position_;
    orientation_ = target_orientation_;
    return std::chrono::ceil<simulation::Duration>(target_time);
  }

  const double linear_step = linear_speed_meters_per_second_ * seconds;
  if (distance <= linear_step) {
    position_ = target_position_;
  } else {
    const double fraction = linear_step / distance;
    position_.x_meters += dx * fraction;
    position_.y_meters += dy * fraction;
    position_.z_meters += dz * fraction;
  }

  const double angular_step = angular_speed_radians_per_second_ * seconds;
  orientation_.roll_radians =
      AdvanceAngle(orientation_.roll_radians, target_orientation_.roll_radians,
                   angular_step);
  orientation_.pitch_radians =
      AdvanceAngle(orientation_.pitch_radians,
                   target_orientation_.pitch_radians, angular_step);
  orientation_.yaw_radians = AdvanceAngle(
      orientation_.yaw_radians, target_orientation_.yaw_radians, angular_step);
  return elapsed;
}

Position DynamicModel::GetPosition() const { return position_; }

Orientation DynamicModel::GetOrientation() const { return orientation_; }

LinearVelocity DynamicModel::GetLinearVelocity() const {
  const double dx = target_position_.x_meters - position_.x_meters;
  const double dy = target_position_.y_meters - position_.y_meters;
  const double dz = target_position_.z_meters - position_.z_meters;
  const double distance = std::hypot(dx, dy, dz);
  if (distance == 0.0 || linear_speed_meters_per_second_ == 0.0) {
    return {};
  }
  return {(dx / distance) * linear_speed_meters_per_second_,
          (dy / distance) * linear_speed_meters_per_second_,
          (dz / distance) * linear_speed_meters_per_second_};
}

AngularVelocity DynamicModel::GetAngularVelocity() const {
  const auto velocity = [this](double current, double target) {
    return current == target ? 0.0
                             : std::copysign(angular_speed_radians_per_second_,
                                             target - current);
  };
  return {velocity(orientation_.roll_radians, target_orientation_.roll_radians),
          velocity(orientation_.pitch_radians, target_orientation_.pitch_radians),
          velocity(orientation_.yaw_radians, target_orientation_.yaw_radians)};
}

double DynamicModel::GetCurrentAmps() const {
  const auto linear = GetLinearVelocity();
  const auto angular = GetAngularVelocity();
  const bool moving = linear.x_meters_per_second != 0.0 ||
                      linear.y_meters_per_second != 0.0 ||
                      linear.z_meters_per_second != 0.0 ||
                      angular.roll_radians_per_second != 0.0 ||
                      angular.pitch_radians_per_second != 0.0 ||
                      angular.yaw_radians_per_second != 0.0;
  return moving ? kMovingCurrentAmps : kIdleCurrentAmps;
}
