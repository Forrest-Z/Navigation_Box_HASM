/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <nie/cv/kd_tree.hpp>
#include <nie/formats/ba_graph/pose_record_kd_tree_adapter.hpp>

using Scalar = double;
using EigenMatrixType = typename Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>;
using KdTreePoseRecord2D = nie::KdTree<double, nie::PoseRecordKdTreeAdapter, 2>;
using KdTreeEigen2D = nie::KdTree<double, nie::EigenKdTreeAdapter<EigenMatrixType>, 2>;
using KdTreeOpenCv2DPoint3_ = nie::KdTree<double, nie::OpenCvKdTreeAdapter<cv::Point3_<Scalar>>, 2>;
using KdTreeOpenCv2DPoint_ = nie::KdTree<double, nie::OpenCvKdTreeAdapter<cv::Point_<Scalar>>, 2>;
using KdTreeOpenCv2DVec = nie::KdTree<double, nie::OpenCvKdTreeAdapter<cv::Vec<Scalar, 3>>, 2>;

class RadiusSearchTest : public ::testing::Test {
public:
    static constexpr size_t kMaxLeafSize = 10;
    static constexpr Scalar kRadius = 1.1;
    using MatchType = typename KdTreePoseRecord2D::MatchType;

    // clang-format off
    std::vector<MatchType> const kExpectedMatches_ {
            {3, 0},
            {2, 1.0},
            {4, 1.0},
    };
    std::vector<std::vector<Scalar>> const kData_{
            {0, 0, 0},
            {1, 1, 1},
            {2, 2, 2},
            {2, 3, 2},
            {2, 4, 2},
            {4, 4, 4},
    };
    // clang-format on
    template <typename TreeType, typename Points>
    void CompareMatches(TreeType kd_tree, Points points) {
        auto matches = kd_tree.RadiusSearch(points[3], kRadius);
        ASSERT_EQ(matches, kExpectedMatches_);
    }
};

class PoseRecordRadiusSearchTest : public RadiusSearchTest {
public:
    std::vector<nie::io::PoseRecord> GetPoses() {
        std::vector<nie::io::PoseRecord> poses(kData_.size());
        nie::io::PoseId pose_id = 0;
        std::transform(
                kData_.cbegin(),
                kData_.cend(),
                poses.begin(),
                [&pose_id](std::vector<Scalar> const& point) -> nie::io::PoseRecord {
                    return {pose_id++,
                            nie::io::PoseRecord::Category::kGps,
                            1,
                            nie::Timestamp_ns(std::chrono::nanoseconds(1)),
                            {{point[0], point[1], point[2]}, {1., 1., 1., 1.}},
                            Eigen::Matrix<double, 6, 6>::Identity()};
                });
        return poses;
    }
};

TEST_F(PoseRecordRadiusSearchTest, PoseRecordAdapter2D) {
    std::vector<nie::io::PoseRecord> const poses = GetPoses();
    nie::PoseRecordKdTreeAdapter adapter(poses);
    KdTreePoseRecord2D kd_tree{adapter, kMaxLeafSize};
    std::vector<KdTreePoseRecord2D::MatchType> matches;
    size_t count = kd_tree.RadiusSearch(poses[3], kRadius, &matches);
    ASSERT_EQ(count, kExpectedMatches_.size());
    ASSERT_EQ(matches, kExpectedMatches_);
}

class EigenMatrixRadiusSearchTest : public RadiusSearchTest {
public:
    EigenMatrixType GetPoints() {
        std::vector<Eigen::Vector3d> vec;

        for (size_t i = 0; i < kData_.size(); ++i) {
            vec.emplace_back(kData_[i].data());
        }
        EigenMatrixType points(vec.data()->data(), 3, vec.size());
        return points;
    }
};

TEST_F(EigenMatrixRadiusSearchTest, EigenAdapter2D) {
    EigenMatrixType const points = GetPoints();
    nie::EigenKdTreeAdapter adapter(points);
    KdTreeEigen2D kd_tree{adapter, kMaxLeafSize};
    std::vector<KdTreeEigen2D::MatchType> matches;
    auto count = kd_tree.RadiusSearch(points.col(3), kRadius, &matches);
    ASSERT_EQ(count, kExpectedMatches_.size());
    ASSERT_EQ(matches, kExpectedMatches_);
}

class OpenCvRadiusSearchTest : public RadiusSearchTest {
public:
    std::vector<cv::Point3_<Scalar>> GetPointsAsPoint3_() {
        std::vector<cv::Point3_<Scalar>> points;

        for (size_t i = 0; i < kData_.size(); ++i) {
            points.emplace_back(kData_[i][0], kData_[i][1], kData_[i][2]);
        }
        return points;
    }

    std::vector<cv::Point_<Scalar>> GetPointsAsPoint_() {
        std::vector<cv::Point_<Scalar>> points;

        for (size_t i = 0; i < kData_.size(); ++i) {
            points.emplace_back(kData_[i][0], kData_[i][1]);
        }
        return points;
    }

    std::vector<cv::Vec<Scalar, 3>> GetPointsAsVec() {
        std::vector<cv::Vec<Scalar, 3>> points;

        for (size_t i = 0; i < kData_.size(); ++i) {
            points.emplace_back(kData_[i][0], kData_[i][1], kData_[i][2]);
        }
        return points;
    }
};

TEST_F(OpenCvRadiusSearchTest, OpenCvAdapter2DPoint_) {
    std::vector<cv::Point_<Scalar>> const points = GetPointsAsPoint_();
    nie::OpenCvKdTreeAdapter<cv::Point_<Scalar>> adapter(points);
    KdTreeOpenCv2DPoint_ kd_tree{adapter, kMaxLeafSize};
    std::vector<KdTreeOpenCv2DPoint_::MatchType> matches;

    auto count = kd_tree.RadiusSearch(points[3], kRadius, &matches);

    ASSERT_EQ(count, kExpectedMatches_.size());
    ASSERT_EQ(matches, kExpectedMatches_);
}

TEST_F(OpenCvRadiusSearchTest, OpenCvAdapter2DPoint3_) {
    std::vector<cv::Point3_<Scalar>> const points = GetPointsAsPoint3_();
    nie::OpenCvKdTreeAdapter<cv::Point3_<Scalar>> adapter(points);
    KdTreeOpenCv2DPoint3_ kd_tree{adapter, kMaxLeafSize};
    std::vector<KdTreeOpenCv2DPoint3_::MatchType> matches;

    auto count = kd_tree.RadiusSearch(points[3], kRadius, &matches);

    ASSERT_EQ(count, kExpectedMatches_.size());
    ASSERT_EQ(matches, kExpectedMatches_);
}

TEST_F(OpenCvRadiusSearchTest, OpenCvAdapter2DVec) {
    std::vector<cv::Vec<Scalar, 3>> const points = GetPointsAsVec();
    nie::OpenCvKdTreeAdapter<cv::Vec<Scalar, 3>> adapter(points);
    KdTreeOpenCv2DVec kd_tree{adapter, kMaxLeafSize};
    std::vector<KdTreeOpenCv2DVec::MatchType> matches;

    auto count = kd_tree.RadiusSearch(points[3], kRadius, &matches);

    ASSERT_EQ(count, kExpectedMatches_.size());
    ASSERT_EQ(matches, kExpectedMatches_);
}
