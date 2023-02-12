/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cmath>

#include <gtest/gtest.h>
#include <nie/hardware/device/pid_controller.hpp>
#include <opencv2/core.hpp>

class PidControllerTest : public testing::Test {
protected:
    double const kInitialValue = 1.0, kTargetValue = 20.0, kTargetValueReverse = -kTargetValue, kTargetError = 0.05,
                 kDeltaT = 1.0;
    uint64_t const rng_seed = 1337;
    nie::PidController controller = nie::PidController(1.0, 1.0, 0.0001, -5.0, 5.0);
};

TEST_F(PidControllerTest, AcessorsAndReset) {
    double current_value = kInitialValue;
    controller.ComputeDelta(kTargetValue - current_value, kDeltaT);
    ASSERT_DOUBLE_EQ(controller.gain_p(), 1.0);
    ASSERT_DOUBLE_EQ(controller.gain_i(), 1.0);
    ASSERT_DOUBLE_EQ(controller.gain_d(), 0.0001);
    ASSERT_NE(controller.error_p(), 0.0);
    ASSERT_NE(controller.error_i(), 0.0);
    ASSERT_NE(controller.error_d(), 0.0);
    controller.Reset();
    ASSERT_EQ(controller.error_p(), 0.0);
    ASSERT_EQ(controller.error_i(), 0.0);
    ASSERT_EQ(controller.error_d(), 0.0);
}

TEST_F(PidControllerTest, ComputeDelta) {
    double current_value = kInitialValue;
    while (abs(kTargetValue - current_value) > kTargetError) {
        auto delta_value = controller.ComputeDelta(kTargetValue - current_value, kDeltaT);
        current_value += delta_value;
    }
    EXPECT_GE(kTargetError, kTargetValue - current_value);
    EXPECT_NEAR(kTargetValue, current_value, kTargetError);
}

TEST_F(PidControllerTest, ComputeDeltaReverse) {
    double current_value = kInitialValue;
    while (abs(kTargetValueReverse - current_value) > kTargetError) {
        auto delta_value = controller.ComputeDelta(kTargetValueReverse - current_value, kDeltaT);
        current_value += delta_value;
    }
    EXPECT_GE(kTargetError, kTargetValueReverse - current_value);
    EXPECT_NEAR(kTargetValueReverse, current_value, kTargetError);
}

TEST_F(PidControllerTest, ComputeDeltaNoisy) {
    double current_value = kInitialValue;
    // seed with a constant -- because we want to add noise, but always the same noise :)
    cv::theRNG().state = static_cast<uint64_t>(rng_seed);
    while (abs(kTargetValue - current_value) > kTargetError) {
        current_value += cv::randu<double>();
        auto delta_value = controller.ComputeDelta(kTargetValue - current_value, kDeltaT);
        current_value += delta_value;
    }
    EXPECT_GE(kTargetError, kTargetValue - current_value);
    EXPECT_NEAR(kTargetValue, current_value, kTargetError);
}
