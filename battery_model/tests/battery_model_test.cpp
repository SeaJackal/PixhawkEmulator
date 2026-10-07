#include "battery_model.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using namespace std::chrono_literals;
constexpr double kTolerance = 1e-12;

TEST(BatteryModelTest, UsesConfiguredMaximumAndStartingCapacity) {
    BatteryModel model{5.0, 2.5};
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.5);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kOk);
    EXPECT_TRUE(model.Process(1.5, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 1.0);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kLow);
    EXPECT_FALSE(model.Process(1.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.5);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
    EXPECT_TRUE(model.Process(-10.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 5.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
    model.Process(1.0, 3600s);
    model.Reset();
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 5.0);
}

TEST(BatteryModelTest, StartingBelowCriticalDoesNotCreateCharge) {
    BatteryModel model{5.0, 0.25};
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
    EXPECT_FALSE(model.Process(0.0, 3600s));
    EXPECT_FALSE(model.Process(1.0, 0s));
    EXPECT_FALSE(model.Process(1.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.25);
    EXPECT_TRUE(model.Process(-1.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 1.25);
}

TEST(BatteryModelTest, CanStartEmpty) {
    BatteryModel model{5.0, 0.0};
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 9.0);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
    EXPECT_THROW(model.Process(1.0, 1s), std::runtime_error);
    model.Reset();
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 5.0);
}

TEST(BatteryModelTest, RejectsInvalidCapacities) {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double maximum : {0.0, -1.0, infinity, -infinity, nan}) {
        EXPECT_DEBUG_DEATH((BatteryModel{maximum, 0.0}), "");
    }
    for (const double start : {-1.0, 5.1, infinity, -infinity, nan}) {
        EXPECT_DEBUG_DEATH((BatteryModel{5.0, start}), "");
    }
}

TEST(BatteryModelTest, StartsFullyCharged) {
    BatteryModel model{2.0, 2.0};
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
}

TEST(BatteryModelTest, DischargesWithLinearVoltage) {
    BatteryModel model{2.0, 2.0};
    EXPECT_TRUE(model.Process(2.0, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 1.0, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
    EXPECT_TRUE(model.Process(1.0, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 0.5, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.9, kTolerance);
}

TEST(BatteryModelTest, StopsDischargeAtCritical) {
    BatteryModel model{2.0, 2.0};
    EXPECT_FALSE(model.Process(3.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.36, kTolerance);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
    EXPECT_FALSE(model.Process(1.0, 3600s));
    EXPECT_FALSE(model.Process(0.0, 0s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.36, kTolerance);
}

TEST(BatteryModelTest, NegativeCurrentRechargesAndClampsAtFull) {
    BatteryModel model{2.0, 2.0};
    model.Process(1.75, 3600s);
    EXPECT_TRUE(model.Process(-0.5, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 0.5, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.9, kTolerance);
    EXPECT_TRUE(model.Process(-3.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
}

TEST(BatteryModelTest, ZeroCurrentOrTimePreservesCharge) {
    BatteryModel model{2.0, 2.0};
    model.Process(1.0, 3600s);
    EXPECT_TRUE(model.Process(0.0, 3600s));
    EXPECT_TRUE(model.Process(2.0, 0s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 1.0);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
}

TEST(BatteryModelTest, ResetRestoresFullBatteryAndAllowsFurtherDischarge) {
    BatteryModel model{2.0, 2.0};
    EXPECT_FALSE(model.Process(3.0, 3600s));
    model.Reset();
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kOk);
    model.Process(1.0, 3600s);
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 1.0);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
}

TEST(BatteryModelTest, SplitTimeStepsMatchSingleTimeStep) {
    BatteryModel single{2.0, 2.0};
    BatteryModel split{2.0, 2.0};
    single.Process(3.0, 600s);
    for (int i = 0; i < 1200; ++i) {
        split.Process(3.0, 500ms);
    }
    EXPECT_NEAR(split.GetChargeAmpHours(), single.GetChargeAmpHours(), kTolerance);
    EXPECT_NEAR(split.GetVoltageVolts(), single.GetVoltageVolts(), kTolerance);
}

TEST(BatteryModelTest, StatusFollowsChargePercentageBoundaries) {
    BatteryModel model{2.0, 2.0};
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kOk);
    model.Process(1.598, 3600s);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kOk);
    model.Reset();
    EXPECT_TRUE(model.Process(1.6, 3600s));
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kLow);
    model.Reset();
    EXPECT_TRUE(model.Process(1.798, 3600s));
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kLow);
    model.Reset();
    EXPECT_FALSE(model.Process(1.8, 3600s));
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
    EXPECT_TRUE(model.Process(-1.0, 3600s));
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kOk);
    EXPECT_FALSE(model.Process(2.0, 3600s));
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
}

TEST(BatteryModelTest, ReturnsFalseWhenStepEndsAtCritical) {
    BatteryModel model{2.0, 2.0};
    EXPECT_FALSE(model.Process(1.0, 6480s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
}

TEST(BatteryModelTest, SplitDischargeAlsoStopsAtCritical) {
    BatteryModel model{2.0, 2.0};
    EXPECT_TRUE(model.Process(1.0, 3600s));
    EXPECT_FALSE(model.Process(1.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
}

}
