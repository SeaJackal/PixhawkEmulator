#include "dynamic_model.h"

#include <gtest/gtest.h>

#include <chrono>

namespace {

using namespace std::chrono_literals;
constexpr double kTolerance = 1e-12;

void ExpectPosition(const DynamicModel& model, const Position& expected);

void ExpectOrientation(const DynamicModel& model, const Orientation& expected);

// -----------------------------------------------------------------------------

void ExpectPosition(const DynamicModel& model, const Position& expected) {
  const auto actual = model.GetPosition();
  EXPECT_NEAR(actual.x_meters, expected.x_meters, kTolerance);
  EXPECT_NEAR(actual.y_meters, expected.y_meters, kTolerance);
  EXPECT_NEAR(actual.z_meters, expected.z_meters, kTolerance);
}

void ExpectOrientation(const DynamicModel& model, const Orientation& expected) {
  const auto actual = model.GetOrientation();
  EXPECT_NEAR(actual.roll_radians, expected.roll_radians, kTolerance);
  EXPECT_NEAR(actual.pitch_radians, expected.pitch_radians, kTolerance);
  EXPECT_NEAR(actual.yaw_radians, expected.yaw_radians, kTolerance);
}

TEST(DynamicModelTest, StartsAtRestAtTheOrigin) {
  DynamicModel model;
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
  model.Process(10s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, ReturnsOnlyTimeUntilCompletePoseArrival) {
  DynamicModel model{2.0, 0.5};
  model.Command({3.0, 4.0, 0.0}, {1.0, -2.0, 0.5});
  EXPECT_EQ(model.Process(10s), 4s);
  ExpectPosition(model, {3.0, 4.0, 0.0});
  ExpectOrientation(model, {1.0, -2.0, 0.5});
  EXPECT_EQ(model.Process(10s), 10s);
  ExpectPosition(model, {3.0, 4.0, 0.0});
  ExpectOrientation(model, {1.0, -2.0, 0.5});
}

TEST(DynamicModelTest, ReturnsFullDurationBeforeAndExactlyAtArrival) {
  DynamicModel model;
  model.Command({3.0, 0.0, 0.0}, {});
  EXPECT_EQ(model.Process(1s), 1s);
  ExpectPosition(model, {1.0, 0.0, 0.0});
  EXPECT_EQ(model.Process(2s), 2s);
  ExpectPosition(model, {3.0, 0.0, 0.0});
}

TEST(DynamicModelTest, ArrivalRoundsUpWithoutExceedingRequestedDuration) {
  DynamicModel model;
  const simulation::Duration tick{1};
  const double tick_seconds = std::chrono::duration<double>(tick).count();
  model.Command({tick_seconds * 0.5, 0.0, 0.0}, {});
  EXPECT_EQ(model.Process(simulation::Duration::zero()),
            simulation::Duration::zero());
  ExpectPosition(model, {});
  EXPECT_EQ(model.Process(tick), tick);
  EXPECT_DOUBLE_EQ(model.GetPosition().x_meters, tick_seconds * 0.5);
  EXPECT_EQ(model.Process(tick * 10), tick * 10);
}

TEST(DynamicModelTest, ReturnsRemainingDurationAfterPartialProgress) {
  DynamicModel model;
  model.Command({0.0, 0.0, 3.0}, {0.0, 1.0, 0.0});
  EXPECT_EQ(model.Process(1s), 1s);
  EXPECT_EQ(model.Process(10s), 2s);
  ExpectPosition(model, {0.0, 0.0, 3.0});
  ExpectOrientation(model, {0.0, 1.0, 0.0});
}

TEST(DynamicModelTest, UnreachablePoseConsumesFullRequestedDuration) {
  DynamicModel model{0.0, 1.0};
  model.Command({1.0, 0.0, 0.0}, {0.0, 0.0, 2.0});
  EXPECT_EQ(model.Process(10s), 10s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {0.0, 0.0, 2.0});
  EXPECT_EQ(model.Process(0s), 0s);
}

TEST(DynamicModelTest, UnreachableRotationConsumesFullRequestedDuration) {
  DynamicModel model{1.0, 0.0};
  model.Command({2.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
  EXPECT_EQ(model.Process(10s), 10s);
  ExpectPosition(model, {2.0, 0.0, 0.0});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, StationarySatisfiedTargetConsumesFullRequestedDuration) {
  DynamicModel model{0.0, 0.0};
  EXPECT_EQ(model.Process(10s), 10s);
  EXPECT_EQ(model.Process(0s), 0s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, CommandDoesNotImmediatelyChangeState) {
  DynamicModel model;
  model.Command({2.0, -3.0, 4.0}, {0.5, -1.0, 2.0});
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, DefaultSpeedsAdvanceOneUnitPerSecond) {
  DynamicModel model;
  model.Command({10.0, 0.0, 0.0}, {10.0, -10.0, 10.0});
  model.Process(1s);
  ExpectPosition(model, {1.0, 0.0, 0.0});
  ExpectOrientation(model, {1.0, -1.0, 1.0});
}

TEST(DynamicModelTest, MovesAlongThreeDimensionalDirectionAtConfiguredSpeed) {
  DynamicModel model{4.0, 1.0};
  // The target is 6 meters away, so half a second covers one third of it.
  model.Command({2.0, -4.0, 4.0}, {});
  model.Process(500ms);
  ExpectPosition(model, {2.0 / 3.0, -4.0 / 3.0, 4.0 / 3.0});
}

TEST(DynamicModelTest, RotatesEachAxisIndependentlyAndClampsShorterRotations) {
  DynamicModel model{1.0, 2.0};
  model.Command({}, {3.0, -4.0, 0.25});
  model.Process(500ms);
  ExpectOrientation(model, {1.0, -1.0, 0.25});
}

TEST(DynamicModelTest, ReachesExactStepTargets) {
  DynamicModel model{2.0, 0.5};
  model.Command({0.0, 0.0, -2.0}, {0.5, -0.5, 0.0});
  model.Process(1s);
  ExpectPosition(model, {0.0, 0.0, -2.0});
  ExpectOrientation(model, {0.5, -0.5, 0.0});
}

TEST(DynamicModelTest, DoesNotOvershootAndRemainsAtTarget) {
  DynamicModel model;
  const Position target{1.0, -2.0, 3.0};
  const Orientation rotation{-0.25, 0.5, -0.75};
  model.Command(target, rotation);
  model.Process(100s);
  ExpectPosition(model, target);
  ExpectOrientation(model, rotation);
  model.Process(100s);
  ExpectPosition(model, target);
  ExpectOrientation(model, rotation);
}

TEST(DynamicModelTest, ZeroElapsedTimePreservesCurrentState) {
  DynamicModel model;
  model.Command({4.0, 0.0, 0.0}, {2.0, -2.0, 2.0});
  model.Process(500ms);
  model.Process(0s);
  ExpectPosition(model, {0.5, 0.0, 0.0});
  ExpectOrientation(model, {0.5, -0.5, 0.5});
}

TEST(DynamicModelTest, ZeroLinearSpeedStillAllowsRotation) {
  DynamicModel model{0.0, 1.0};
  model.Command({3.0, 4.0, 5.0}, {2.0, -2.0, 2.0});
  model.Process(1s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {1.0, -1.0, 1.0});
}

TEST(DynamicModelTest, ZeroAngularSpeedStillAllowsTranslation) {
  DynamicModel model{1.0, 0.0};
  model.Command({0.0, -4.0, 0.0}, {2.0, -2.0, 2.0});
  model.Process(1s);
  ExpectPosition(model, {0.0, -1.0, 0.0});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, BothSpeedsCanBeZero) {
  DynamicModel model{0.0, 0.0};
  // Zero distance and zero step must not cause division by zero.
  model.Process(0s);
  model.Process(10s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
  model.Command({1.0, 2.0, 3.0}, {1.0, 2.0, 3.0});
  model.Process(10s);
  ExpectPosition(model, {});
  ExpectOrientation(model, {});
}

TEST(DynamicModelTest, NewCommandRedirectsMotionFromCurrentState) {
  DynamicModel model;
  model.Command({4.0, 0.0, 0.0}, {4.0, -4.0, 4.0});
  model.Process(1s);
  model.Command({1.0, -4.0, 0.0}, {-4.0, 4.0, 1.0});
  model.Process(500ms);
  ExpectPosition(model, {1.0, -0.5, 0.0});
  ExpectOrientation(model, {0.5, -0.5, 1.0});
}

TEST(DynamicModelTest, ReportsVelocityTowardTargetAndStopsOnArrival) {
  DynamicModel model{3.0, 2.0};
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kIdleCurrentAmps);
  model.Command({2.0, -4.0, 4.0}, {1.0, -4.0, 0.0});
  const auto linear = model.GetLinearVelocity();
  EXPECT_NEAR(linear.x_meters_per_second, 1.0, kTolerance);
  EXPECT_NEAR(linear.y_meters_per_second, -2.0, kTolerance);
  EXPECT_NEAR(linear.z_meters_per_second, 2.0, kTolerance);
  const auto angular = model.GetAngularVelocity();
  EXPECT_DOUBLE_EQ(angular.roll_radians_per_second, 2.0);
  EXPECT_DOUBLE_EQ(angular.pitch_radians_per_second, -2.0);
  EXPECT_DOUBLE_EQ(angular.yaw_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kMovingCurrentAmps);

  model.Process(500ms);
  EXPECT_DOUBLE_EQ(model.GetAngularVelocity().roll_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetAngularVelocity().pitch_radians_per_second, -2.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kMovingCurrentAmps);

  model.Process(10s);
  const auto stopped_linear = model.GetLinearVelocity();
  const auto stopped_angular = model.GetAngularVelocity();
  EXPECT_DOUBLE_EQ(stopped_linear.x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(stopped_linear.y_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(stopped_linear.z_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(stopped_angular.roll_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(stopped_angular.pitch_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(stopped_angular.yaw_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kIdleCurrentAmps);
}

TEST(DynamicModelTest, RotationAloneDrawsMovingCurrent) {
  DynamicModel model;
  model.Command({}, {0.0, 0.0, -1.0});
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kMovingCurrentAmps);
  model.Process(1s);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kIdleCurrentAmps);
}

TEST(DynamicModelTest, DisabledMotionDrawsIdleCurrentDespitePendingTarget) {
  DynamicModel model{0.0, 0.0};
  model.Command({1.0, 2.0, 3.0}, {-1.0, -2.0, -3.0});
  const auto linear = model.GetLinearVelocity();
  const auto angular = model.GetAngularVelocity();
  EXPECT_DOUBLE_EQ(linear.x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(linear.y_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(linear.z_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(angular.roll_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(angular.pitch_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(angular.yaw_radians_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kIdleCurrentAmps);
}

TEST(DynamicModelTest, VelocityUpdatesImmediatelyWhenRedirectedOrCancelled) {
  DynamicModel model;
  model.Command({4.0, 0.0, 0.0}, {});
  model.Process(1s);
  model.Command({-4.0, 0.0, 0.0}, {});
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, -1.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kMovingCurrentAmps);
  model.Command(model.GetPosition(), model.GetOrientation());
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 0.0);
  EXPECT_DOUBLE_EQ(model.GetCurrentAmps(), DynamicModel::kIdleCurrentAmps);
}

TEST(DynamicModelTest, SplitTimeStepsMatchSingleTimeStep) {
  DynamicModel single{2.0, 0.5};
  DynamicModel split{2.0, 0.5};
  for (auto* model : {&single, &split}) {
    model->Command({3.0, -6.0, 6.0}, {0.25, -2.0, 3.0});
  }
  single.Process(2s);
  for (int i = 0; i < 8; ++i) {
    split.Process(250ms);
  }
  ExpectPosition(split, single.GetPosition());
  ExpectOrientation(split, single.GetOrientation());
}

}  // namespace
