/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/lidar/geometry/helper_area.hpp>

// Just a sanity check so we know nobody touched the math.
TEST(HelperAreaTest, IntersectionArea) {
constexpr double kIntersectionArea = 14671.688;
constexpr double kIntersectionIou = 0.15111567;

    nie::Isometry3qd pose1{
            Eigen::Vector3d{639874.7534, 3457155.489, 16.93053511},
            Eigen::Quaterniond{-0.6846969192, -0.1609459397, 0.6933829716, -0.1565458016}};
    nie::Isometry3qd pose2{
            Eigen::Vector3d{639705.7791, 3457071.693, 16.8199323},
            Eigen::Quaterniond{-0.6797929193, -0.1597309331, 0.696052492, -0.1669686925}};
    nie::Bboxf box1{{-33.98311996, -101.5319977, -111.8314743}, {10.79384518, 103.4732819, 167.2658081}};
    nie::Bboxf box2{{-24.32649994, -104.841011, -94.40475464}, {5.856451988, 100.3891907, 171.4776306}};
    nie::PoseBbox bounds1{pose1, box1};
    nie::PoseBbox bounds2{pose2, box2};

    // fails on area
    EXPECT_FALSE(nie::CheckIntersectionArea(bounds1, bounds2, kIntersectionArea + 1, kIntersectionIou - 0.1));
    // fails on ratio
    EXPECT_FALSE(nie::CheckIntersectionArea(bounds1, bounds2, kIntersectionArea - 1, kIntersectionIou + 0.1));
    // works!
    EXPECT_TRUE(nie::CheckIntersectionArea(bounds1, bounds2, kIntersectionArea - 1, kIntersectionIou - 0.1));
}