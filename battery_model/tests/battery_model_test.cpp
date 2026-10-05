#include "battery_model.h"

#include <gtest/gtest.h>

namespace {

using namespace std::chrono_literals;
constexpr double kTolerance = 1e-12;

TEST(BatteryModelTest, StartsFullyCharged) {
    BatteryModel model;
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
}

TEST(BatteryModelTest, DischargesWithLinearVoltage) {
    BatteryModel model;
    EXPECT_TRUE(model.Process(2.0, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 1.0, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
    EXPECT_TRUE(model.Process(1.0, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 0.5, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.9, kTolerance);
}

TEST(BatteryModelTest, StopsDischargeAtCritical) {
    BatteryModel model;
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
    BatteryModel model;
    model.Process(1.75, 3600s);
    EXPECT_TRUE(model.Process(-0.5, 1800s));
    EXPECT_NEAR(model.GetChargeAmpHours(), 0.5, kTolerance);
    EXPECT_NEAR(model.GetVoltageVolts(), 9.9, kTolerance);
    EXPECT_TRUE(model.Process(-3.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 2.0);
    EXPECT_DOUBLE_EQ(model.GetVoltageVolts(), 12.6);
}

TEST(BatteryModelTest, ZeroCurrentOrTimePreservesCharge) {
    BatteryModel model;
    model.Process(1.0, 3600s);
    EXPECT_TRUE(model.Process(0.0, 3600s));
    EXPECT_TRUE(model.Process(2.0, 0s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 1.0);
    EXPECT_NEAR(model.GetVoltageVolts(), 10.8, kTolerance);
}

TEST(BatteryModelTest, ResetRestoresFullBatteryAndAllowsFurtherDischarge) {
    BatteryModel model;
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
    BatteryModel single;
    BatteryModel split;
    single.Process(3.0, 600s);
    for (int i = 0; i < 1200; ++i) {
        split.Process(3.0, 500ms);
    }
    EXPECT_NEAR(split.GetChargeAmpHours(), single.GetChargeAmpHours(), kTolerance);
    EXPECT_NEAR(split.GetVoltageVolts(), single.GetVoltageVolts(), kTolerance);
}

TEST(BatteryModelTest, StatusFollowsChargePercentageBoundaries) {
    BatteryModel model;
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
    BatteryModel model;
    EXPECT_FALSE(model.Process(1.0, 6480s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
}

TEST(BatteryModelTest, SplitDischargeAlsoStopsAtCritical) {
    BatteryModel model;
    EXPECT_TRUE(model.Process(1.0, 3600s));
    EXPECT_FALSE(model.Process(1.0, 3600s));
    EXPECT_DOUBLE_EQ(model.GetChargeAmpHours(), 0.2);
    EXPECT_EQ(model.GetStatus(), BatteryModel::Status::kCritical);
}

}
