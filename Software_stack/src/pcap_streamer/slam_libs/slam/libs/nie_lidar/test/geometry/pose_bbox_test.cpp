///* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
// * Information classification: Confidential
// * This content is protected by international copyright laws.
// * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/lidar/geometry/bbox.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>

TEST(PoseBboxTest, CopyWithOrigin) {
    nie::PoseBbox posebox{
            {0, 0, 0},
            {{
                     1,
                     2,
                     1,
             },
             {3, 3, 3}}};
    Eigen::Vector3d translation = {1, 1, 1};
    posebox = posebox.CopyWithOrigin(translation);

    ASSERT_DOUBLE_EQ(posebox.bbox().min.x(), 0);
    ASSERT_DOUBLE_EQ(posebox.bbox().min.y(), 1);
    ASSERT_DOUBLE_EQ(posebox.bbox().min.z(), 0);
    ASSERT_DOUBLE_EQ(posebox.bbox().max.x(), 2);
    ASSERT_DOUBLE_EQ(posebox.bbox().max.y(), 2);
    ASSERT_DOUBLE_EQ(posebox.bbox().max.z(), 2);
}

TEST(PoseBboxTest, UpdateBbox) {
    nie::PoseBbox posebox{
            {3, 0, 0},
            {{
                     -1,
                     -2,
                     -1,
             },
             {3, 3, 3}}};

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1, 2, 7));
    cloud.push_back(pcl::PointXYZ(-1, 0, 0));
    cloud.push_back(pcl::PointXYZ(5, 4, 2));
    cloud.push_back(pcl::PointXYZ(-9, -5, 6));

    posebox.UpdateBbox(cloud);

    ASSERT_FLOAT_EQ(posebox.bbox().min.x(), -9);
    ASSERT_FLOAT_EQ(posebox.bbox().min.y(), -5);
    ASSERT_FLOAT_EQ(posebox.bbox().min.z(), 0);
    ASSERT_FLOAT_EQ(posebox.bbox().max.x(), 5);
    ASSERT_FLOAT_EQ(posebox.bbox().max.y(), 4);
    ASSERT_FLOAT_EQ(posebox.bbox().max.z(), 7);
}

TEST(PoseBboxTest, Contains) {
    nie::PoseBbox posebox{
            {0, 0, 0},
            {{
                     -1,
                     -2,
                     -1,
             },
             {3, 3, 3}}};
    Eigen::Vector3d point1 = {1, 1, 1};
    Eigen::Vector3d point2 = {-4, 2, 1};

    ASSERT_TRUE(posebox.Contains(point1));
    ASSERT_FALSE(posebox.Contains(point2));
}

TEST(PoseBboxTest, Intersection2D) {
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

    double area;
    double iou;
    bounds1.Intersection2D(bounds2, &area, &iou);

    EXPECT_FLOAT_EQ(kIntersectionArea, area);
    EXPECT_FLOAT_EQ(kIntersectionIou, iou);
}
