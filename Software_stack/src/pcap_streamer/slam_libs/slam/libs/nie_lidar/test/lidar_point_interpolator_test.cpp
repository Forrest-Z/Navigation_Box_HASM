/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/lidar/io/lidar_stream_consumer/lidar_point_interpolator.hpp>

class LidarPointInterpolatorF : public ::testing::Test {
protected:
    constexpr static double kPrecision = 1e-12;

    //clang-format off
    std::vector<nie::io::PoseRecord> const kPoses{
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(0)),
             {{0, 0, 0}, {0, 0, 0, 0}},
             {}},
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(1)),
             {{1, 1, 1}, {1, 1, 1, 1}},
             {}},
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(2)),
             {{2, 2, 2}, {2, 2, 2, 2}},
             {}},
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(4)),
             {{3, 3, 3}, {3, 3, 3, 3}},
             {}},
            {3,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(8)),
             {{4, 4, 4}, {4, 4, 4, 4}},
             {}},
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(16)),
             {{5, 5, 5}, {5, 5, 5, 5}},
             {}},
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(32)),
             {{6, 6, 6}, {6, 6, 6, 6}},
             {}},
    };
    //clang-format on
};

TEST_F(LidarPointInterpolatorF, SameValue) {
    nie::LidarPointInterpolator lidar_point_interpolator{kPoses.cbegin(), kPoses.cend()};

    for (auto const& pose_record : kPoses) {
        auto [isometry, success] = lidar_point_interpolator(pose_record.timestamp);

        ASSERT_TRUE(success);

        EXPECT_NEAR(isometry.translation().x(), pose_record.isometry.translation().x(), kPrecision);
        EXPECT_NEAR(isometry.translation().y(), pose_record.isometry.translation().y(), kPrecision);
        EXPECT_NEAR(isometry.translation().z(), pose_record.isometry.translation().z(), kPrecision);
        EXPECT_NEAR(isometry.rotation().x(), pose_record.isometry.rotation().x(), kPrecision);
        EXPECT_NEAR(isometry.rotation().y(), pose_record.isometry.rotation().y(), kPrecision);
        EXPECT_NEAR(isometry.rotation().z(), pose_record.isometry.rotation().z(), kPrecision);
        EXPECT_NEAR(isometry.rotation().w(), pose_record.isometry.rotation().w(), kPrecision);
    }
}
