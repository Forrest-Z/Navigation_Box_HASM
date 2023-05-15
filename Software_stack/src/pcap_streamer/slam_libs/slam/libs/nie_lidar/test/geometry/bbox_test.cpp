///* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
// * Information classification: Confidential
// * This content is protected by international copyright laws.
// * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/lidar/geometry/bbox.hpp>

TEST(BboxTest, Create) {
    nie::Bboxf box{};

    pcl::PointCloud<pcl::PointXYZ> cloud;
    nie::Isometry3qd const isom{{1, 0, 0}, {0, 0, 1, 0}};

    cloud.push_back(pcl::PointXYZ(1, 2, 7));
    cloud.push_back(pcl::PointXYZ(-1, 0, 0));
    cloud.push_back(pcl::PointXYZ(5, 4, 2));
    cloud.push_back(pcl::PointXYZ(-9, -5, 6));
    nie::Bboxf box_cloud = box.Create(cloud, isom);

    ASSERT_FLOAT_EQ(box_cloud.min.x(), -4);
    ASSERT_FLOAT_EQ(box_cloud.min.y(), -5);
    ASSERT_FLOAT_EQ(box_cloud.min.z(), -7);
    ASSERT_FLOAT_EQ(box_cloud.max.x(), 10);
    ASSERT_FLOAT_EQ(box_cloud.max.y(), 4);
    ASSERT_FLOAT_EQ(box_cloud.max.z(), 0);
}

TEST(BboxTest, MaxBoundingBox) {
    float min_f = std::numeric_limits<float>::lowest();
    float max_f = std::numeric_limits<float>::max();
    ASSERT_EQ(nie::Bbox<float>::MaxBoundingBox().min.x(), min_f);
    ASSERT_EQ(nie::Bbox<float>::MaxBoundingBox().max.x(), max_f);

    double min_d = std::numeric_limits<double>::lowest();
    double max_d = std::numeric_limits<double>::max();
    ASSERT_EQ(nie::Bbox<double>::MaxBoundingBox().min.x(), min_d);
    ASSERT_EQ(nie::Bbox<double>::MaxBoundingBox().max.x(), max_d);
}

TEST(BboxTest, InverseMaxBoundingBox) {
    float min_f = std::numeric_limits<float>::lowest();
    float max_f = std::numeric_limits<float>::max();
    ASSERT_EQ(nie::Bbox<float>::InverseMaxBoundingBox().min.x(), max_f);
    ASSERT_EQ(nie::Bbox<float>::InverseMaxBoundingBox().max.x(), min_f);

    double min_d = std::numeric_limits<double>::lowest();
    double max_d = std::numeric_limits<double>::max();
    ASSERT_EQ(nie::Bbox<double>::InverseMaxBoundingBox().min.x(), max_d);
    ASSERT_EQ(nie::Bbox<double>::InverseMaxBoundingBox().max.x(), min_d);
}

TEST(BboxTest, Inflate) {
    nie::Bboxf box{{1, 1, 1}, {2, 2, 2}};
    box.Inflate({2, 3, 4});

    ASSERT_FLOAT_EQ(box.min.x(), -1);
    ASSERT_FLOAT_EQ(box.min.y(), -2);
    ASSERT_FLOAT_EQ(box.min.z(), -3);
    ASSERT_FLOAT_EQ(box.max.x(), 4);
    ASSERT_FLOAT_EQ(box.max.y(), 5);
    ASSERT_FLOAT_EQ(box.max.z(), 6);
}

TEST(BboxTest, FitPoint) {
    nie::Bboxf box{{1, 1, 1}, {2, 2, 2}};
    pcl::PointXYZ point(3, 2, 4);
    box.FitPoint(point);

    ASSERT_TRUE(box.Contains(point));
    ASSERT_FLOAT_EQ(box.max.x(), 3);
    ASSERT_FLOAT_EQ(box.max.y(), 2);
    ASSERT_FLOAT_EQ(box.max.z(), 4);
}

TEST(BboxTest, Volume) {
    nie::Bboxf box{{1, 1, 1}, {2, 3, 2}};

    ASSERT_FLOAT_EQ(box.Volume(), 2.0);
}

TEST(BboxTest, Range) {
    nie::Bboxf box{{1, 1, 1}, {2, 3, 2}};

    ASSERT_FLOAT_EQ(box.Range().x(), 1);
    ASSERT_FLOAT_EQ(box.Range().y(), 2);
    ASSERT_FLOAT_EQ(box.Range().z(), 1);
}

TEST(BboxTest, Center) {
    nie::Bboxf box{{-7, 4, 6}, {-4, 9, 8}};

    ASSERT_FLOAT_EQ(box.Center().x(), -5.5);
    ASSERT_FLOAT_EQ(box.Center().y(), 6.5);
    ASSERT_FLOAT_EQ(box.Center().z(), 7);
}

TEST(BboxTest, Contains) {
    nie::Bboxf box{{1, 1, 1}, {2, 2, 2}};
    Eigen::Vector3f point = {1.5, 1.5, 1.5};

    ASSERT_TRUE(box.Contains(point));
}

TEST(BboxTest, GetAllCorners) {
    nie::Bboxf box{{1, 2, 1}, {3, 3, 3}};

    std::array<Eigen::Vector3f, 8> corners = box.GetAllCorners();

    ASSERT_FLOAT_EQ(corners[0].x(), 1);
    ASSERT_FLOAT_EQ(corners[0].y(), 2);
    ASSERT_FLOAT_EQ(corners[0].z(), 1);

    ASSERT_FLOAT_EQ(corners[1].x(), 1);
    ASSERT_FLOAT_EQ(corners[1].y(), 2);
    ASSERT_FLOAT_EQ(corners[1].z(), 3);

    ASSERT_FLOAT_EQ(corners[2].x(), 1);
    ASSERT_FLOAT_EQ(corners[2].y(), 3);
    ASSERT_FLOAT_EQ(corners[2].z(), 1);

    ASSERT_FLOAT_EQ(corners[3].x(), 3);
    ASSERT_FLOAT_EQ(corners[3].y(), 2);
    ASSERT_FLOAT_EQ(corners[3].z(), 1);

    ASSERT_FLOAT_EQ(corners[4].x(), 1);
    ASSERT_FLOAT_EQ(corners[4].y(), 3);
    ASSERT_FLOAT_EQ(corners[4].z(), 3);

    ASSERT_FLOAT_EQ(corners[5].x(), 3);
    ASSERT_FLOAT_EQ(corners[5].y(), 2);
    ASSERT_FLOAT_EQ(corners[5].z(), 3);

    ASSERT_FLOAT_EQ(corners[6].x(), 3);
    ASSERT_FLOAT_EQ(corners[6].y(), 3);
    ASSERT_FLOAT_EQ(corners[6].z(), 1);

    ASSERT_FLOAT_EQ(corners[7].x(), 3);
    ASSERT_FLOAT_EQ(corners[7].y(), 3);
    ASSERT_FLOAT_EQ(corners[7].z(), 3);
}

TEST(BboxTest, Operators) {
    nie::Bboxf box1{{1, 3, 1}, {5, 5, 5}};
    nie::Bboxf box2{{2, 2, 2}, {4, 4, 4}};
    nie::Bboxf box_temp;

    box_temp = box1;
    box_temp &= box2;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 4);

    box_temp = box1 & box2;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 4);

    nie::Bboxf box1_no_overlap{{0, 0, 0}, {1, 2, 1}};
    nie::Bboxf box2_no_overlap{{2, 3, 2}, {4, 4, 4}};
    nie::Bboxf box_temp_no_overlap;

    box_temp_no_overlap = box1_no_overlap;
    box_temp_no_overlap &= box2_no_overlap;
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.y(), 3);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.z(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.x(), 1);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.y(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.z(), 1);

    box_temp_no_overlap = box1_no_overlap;
    box_temp_no_overlap = box1_no_overlap & box2_no_overlap;
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.y(), 3);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.min.z(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.x(), 1);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.y(), 2);
    ASSERT_FLOAT_EQ(box_temp_no_overlap.max.z(), 1);

    box_temp_no_overlap = box1_no_overlap;
    box_temp_no_overlap &= box2_no_overlap;

    box_temp = box1;
    box_temp |= box2;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 1);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 5);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 5);

    box_temp = box1 | box2;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 1);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 5);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 5);

    pcl::PointXYZ point(1, 2, 3);
    box_temp = box1;
    box_temp += point;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 6);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 7);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 8);

    box_temp = box1 + point;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 6);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 7);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 8);

    box_temp = box1;
    box_temp -= point;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 0);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.z(), -2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 2);

    box_temp = box1 - point;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 0);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.z(), -2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 2);

    Eigen::Vector3f point_eig(1, 2, 3);
    box_temp = box1;
    box_temp += point_eig;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 6);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 7);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 8);

    box_temp = box1 + point_eig;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 2);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 5);
    ASSERT_FLOAT_EQ(box_temp.min.z(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 6);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 7);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 8);

    box_temp = box1;
    box_temp -= point_eig;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 0);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.z(), -2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 2);

    box_temp = box1 - point_eig;
    ASSERT_FLOAT_EQ(box_temp.min.x(), 0);
    ASSERT_FLOAT_EQ(box_temp.min.y(), 1);
    ASSERT_FLOAT_EQ(box_temp.min.z(), -2);
    ASSERT_FLOAT_EQ(box_temp.max.x(), 4);
    ASSERT_FLOAT_EQ(box_temp.max.y(), 3);
    ASSERT_FLOAT_EQ(box_temp.max.z(), 2);
}