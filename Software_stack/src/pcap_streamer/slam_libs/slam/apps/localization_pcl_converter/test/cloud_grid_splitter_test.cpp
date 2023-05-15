/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <cloud_grid_splitter.hpp>

class CloudGridSplitterF : public ::testing::Test {
protected:
    constexpr static uint kNumX = 10;
    constexpr static uint kNumY = 10;
    constexpr static double kDistPoints = 50.0;
};

TEST_F(CloudGridSplitterF, EmptyCloud) {
    nie::Cloud<pcl::PointXYZ> cloud;
    nie::CloudGridSplitter<pcl::PointXYZ> cloud_splitter(kDistPoints);
    std::vector<nie::Cloud<pcl::PointXYZ>> vec = cloud_splitter.Split(std::move(cloud));
    EXPECT_TRUE(vec.empty());
}

TEST_F(CloudGridSplitterF, SplitOutputSize) {
    nie::Cloud<pcl::PointXYZ> cloud;
    nie::CloudGridSplitter<pcl::PointXYZ> cloud_splitter(kDistPoints);
    // Add small distance to each point to avoid edge cases where point is exactly on edge of grid.
    float delta_distance = 1e-4;
    for (uint px = 0; px < kNumX; ++px) {
        for (uint py = 0; py < kNumY; ++py) {
            cloud.point_cloud_ptr()->points.push_back(
                    pcl::PointXYZ(px * (kDistPoints + delta_distance), py * (kDistPoints + delta_distance), 0.0));
        }
    }
    // Create vector with a point missing, which will result in an empty point cloud which should be removed.
    nie::Cloud<pcl::PointXYZ> cloud_with_empty = cloud.Copy();
    cloud_with_empty.point_cloud().points.pop_back();
    // Test full cloud
    auto vec = cloud_splitter.Split(std::move(cloud));
    EXPECT_EQ(vec.size(), kNumX * kNumY);
    // Test cloud with point missing
    vec = cloud_splitter.Split(std::move(cloud_with_empty));
    EXPECT_EQ(vec.size(), kNumX * kNumY - 1);
}