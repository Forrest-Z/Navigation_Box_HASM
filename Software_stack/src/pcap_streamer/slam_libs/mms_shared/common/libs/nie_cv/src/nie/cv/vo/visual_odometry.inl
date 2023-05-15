/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cassert>
#include <type_traits>

#include <glog/logging.h>
#include <nie/core/scoped_timer.hpp>
#include <opencv2/core/eigen.hpp>

#include "feature/match.hpp"
#include "nie/cv/geometry/triangulation.hpp"
#include "types.hpp"

namespace nie {

namespace detail {

inline void LogRate(cv::Mat const& inliers, std::string const& subject) {
    if (VLOG_IS_ON(4)) {
        int sum_mask = static_cast<int>(cv::sum(inliers)[0]);
        double rate = (sum_mask == 0 ? 0. : double(sum_mask) / inliers.rows * 100.);
        VLOG(4) << "Matches used in calculating the " << subject << ": " << sum_mask << " out of " << inliers.rows
                << " = " << rate << "%" << std::endl;
    }
}

}  // namespace detail

template <typename Derived, typename Rotation>
bool EstimateMotion(
        KeypointVector const& features_a,
        KeypointVector const& features_b,
        cv::Matx33d const& K,
        MatchVector* p_matches,
        Isometry3Base<Derived, Rotation>* p_motion) {
    ScopedTimer timer("visual_odometry.inl::EstimateMotion");
    assert(p_matches != nullptr);
    MatchVector& matches = *p_matches;

    // Sanity checks
    if (matches.size() < 5 or features_a.empty() or features_b.empty()) {
        return false;
    }

    // Prepare inputs for calculation of the essential matrix
    KeypointVector points_a, points_b;
    points_a.reserve(matches.size());
    points_b.reserve(matches.size());
    for (FeatureMatch const& match : matches) {
        points_a.emplace_back(features_a[match.index_a]);
        points_b.emplace_back(features_b[match.index_b]);
    }

    cv::Mat inliers;  // filled by findEssentialMat: out- (0) / inlier (1)

    // Actually calculate the essential matrix
    cv::Matx33d E = cv::findEssentialMat(points_a, points_b, K, cv::RANSAC, 0.999, 1., inliers);

    detail::LogRate(inliers, "essential matrix");

    // Calculate the camera motion based on the essential matrix
    cv::Mat R, t;
    int inlier_count = cv::recoverPose(E, points_a, points_b, K, R, t, inliers);

    detail::LogRate(inliers, "camera motion");

    if (inlier_count > 0) {
        // FIXME(jbr): Remove the copy. Either change FilterMatchVector to use a mask (because typically estimation
        // FIXME(jbr): methods generate a mask, not a filter) or change the filter interface to use an evaluation
        // FIXME(jbr): function.
        // Only keep the inlier matches, they were actually used to calculate the motion
        std::vector<bool> filter(inliers.rows);
        for (std::size_t i = 0; i < static_cast<std::size_t>(inliers.rows); ++i) {
            filter[i] = inliers.at<unsigned char>(i) == 0;  // outliers (inliers = 0) should be removed (filter = true)
        }
        FilterMatchVector(filter, p_matches);

        cv::cv2eigen(t, p_motion->translation());
        cv::cv2eigen(R, p_motion->rotation());

        // The transformation is returned as world->camera, but it should be the convention camera->world, so the
        // inverse
        p_motion->Inverse();

        return true;
    }

    return false;
}

// TODO (MvB): Check if copying of points can be avoided
template <typename Isometry>
void TriangulateFeatures(
        std::vector<Isometry> const& poses,
        std::vector<KeypointVector> const& features,
        std::vector<std::vector<std::size_t>> const& matched_features,
        Eigen::Matrix3d const& K,
        std::vector<Eigen::Vector3d>* p_objects) {
    ScopedTimer timer("visual_odometry.inl::TriangulateFeatures");
    assert(p_objects != nullptr);
    assert(poses.size() == features.size());

    std::vector<Eigen::Vector3d>& objects = *p_objects;

    // Convert the poses from quaternion to matrix for calculation efficiency
    std::vector<Isometry3md> const poses_m(poses.begin(), poses.end());

    objects.clear();
    objects.reserve(matched_features.size());

    for (std::vector<std::size_t> const& matched_feature : matched_features) {
        assert(features.size() == matched_feature.size());

        std::vector<Eigen::Vector2f> image_points(features.size());
        for (std::size_t index = 0; index < features.size(); ++index) {
            Keypoint const& p = features[index][matched_feature[index]];
            image_points[index] = {p.x, p.y};
        }

        objects.emplace_back();
        TriangulateLinear(poses_m, image_points, K, &objects.back());
    }
}

}  // namespace nie
