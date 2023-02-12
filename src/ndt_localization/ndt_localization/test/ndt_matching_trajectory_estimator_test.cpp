/** ------------------------------------------------------------------------------------
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 * ------------------------------------------------------------------------------------*
 */

#include "ndt_matching_trajectory_estimator.hpp"

#include <gtest/gtest.h>

static void AssertNdtPosesEqual(nie::ndt::Pose const& lhs, nie::ndt::Pose const& rhs) {
    ASSERT_DOUBLE_EQ(lhs.x, rhs.x);
    ASSERT_DOUBLE_EQ(lhs.y, rhs.y);
    ASSERT_DOUBLE_EQ(lhs.z, rhs.z);
    ASSERT_DOUBLE_EQ(lhs.roll, rhs.roll);
    ASSERT_DOUBLE_EQ(lhs.pitch, rhs.pitch);
    ASSERT_DOUBLE_EQ(lhs.yaw, rhs.yaw);
}

class NdtMatchingTrajectoryEstimator : public testing::Test {
protected:
    nie::ndt::Pose const P_0{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    nie::ndt::Pose const P_1{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    nie::ndt::Pose const P_2{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    nie::ndt::Pose const P_3{2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    nie::ndt::TrajectoryEstimator trajectory_estimator;

    // Our pose prediction functions ignore acceleration effects (but not
    // velocity) for Z and Yaw and do not update roll and pitch at all. Which is
    // why these do not vary in he same way, given the same initial value.
    nie::ndt::Pose const kControlP4{3.5, 3.5, 3.0, 2.0, 2.0, 3.0};
    nie::ndt::Pose const kControlP4Lag1{2.0, 2.0, 2.0, 1.0, 1.0, 2.0};

    double const kDeltaTime = 0.5;
    unsigned int const kMaxValidLag = 8;
};

TEST_F(NdtMatchingTrajectoryEstimator, PredictNoLag) {
    trajectory_estimator.Update(P_0, kDeltaTime);
    trajectory_estimator.Update(P_1, kDeltaTime);
    trajectory_estimator.Update(P_2, kDeltaTime);
    trajectory_estimator.Update(P_3, kDeltaTime);

    nie::ndt::Pose const P_4 = trajectory_estimator.EstimateLatest();
    AssertNdtPosesEqual(P_4, kControlP4);
}

TEST_F(NdtMatchingTrajectoryEstimator, PredictSmallHistory) {
    trajectory_estimator.Update(P_0, kDeltaTime);
    trajectory_estimator.Update(P_1, kDeltaTime);

    // When we don't have enough history to estimate, we return the latest
    // pose we have.
    nie::ndt::Pose const P_3_no_history = trajectory_estimator.EstimateLatest();
    AssertNdtPosesEqual(P_3_no_history, P_1);

    trajectory_estimator.Update(P_2, kDeltaTime);
    trajectory_estimator.Update(P_3, kDeltaTime);

    nie::ndt::Pose const P_4 = trajectory_estimator.EstimateLatest();
    AssertNdtPosesEqual(P_4, kControlP4);
}

TEST_F(NdtMatchingTrajectoryEstimator, PredictLag1) {
    trajectory_estimator.Update(P_0, kDeltaTime);
    trajectory_estimator.Update(P_1, kDeltaTime);
    trajectory_estimator.Update(P_2, kDeltaTime);
    trajectory_estimator.Update(P_3, kDeltaTime);

    nie::ndt::Pose const P_4 = trajectory_estimator.EstimateWithLag(1);
    AssertNdtPosesEqual(P_4, kControlP4Lag1);
}

TEST_F(NdtMatchingTrajectoryEstimator, DeathTests) {
    trajectory_estimator.Update(P_0, kDeltaTime);
    trajectory_estimator.Update(P_1, kDeltaTime);
    trajectory_estimator.Update(P_2, kDeltaTime);
    trajectory_estimator.Update(P_3, kDeltaTime);

    ASSERT_NO_FATAL_FAILURE(nie::ndt::Pose const P_4 = trajectory_estimator.EstimateWithLag(kMaxValidLag));
    ASSERT_DEATH(
            nie::ndt::Pose const P_4 = trajectory_estimator.EstimateWithLag(kMaxValidLag + 1),
            "parameter can not be >= history size");
}
