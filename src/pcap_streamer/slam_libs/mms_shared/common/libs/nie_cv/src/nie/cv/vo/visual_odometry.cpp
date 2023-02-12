/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "visual_odometry.hpp"

#include <cassert>
#include <random>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/spatial_grid.hpp>

#include "ba_problem.hpp"

namespace nie {

namespace detail {

float CalcFlowAverage(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<std::size_t> const& match_indices) {
    if (match_indices.empty()) {
        return 0;
    }

    Eigen::Vector2f sum{0, 0};
    for (std::size_t const& idx : match_indices) {
        FeatureMatch const& match = matches[idx];
        sum += nie::ConvertPoint(features[match.index_b] - prev_features[match.index_a]);
    }
    return sum.norm() / match_indices.size();
}

// Generate n random number pairs between min and max (exclusive), without the same number in a pair.
// Duplicate pairs are possible.
std::vector<std::pair<std::size_t, std::size_t>> GetRandomPairs(std::size_t min, std::size_t max, std::size_t n) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<std::size_t> random(min, max - 1);

    std::vector<std::pair<std::size_t, std::size_t>> result;
    result.reserve(n);
    while (result.size() != n) {
        std::size_t a = random(mt);
        std::size_t b = random(mt);
        if (a != b) {
            result.emplace_back(a, b);
        }
    }
    return result;
}

}  // namespace detail

SpatialGridVo CreateSpatialGridVo(
        std::size_t const cell_size, std::vector<Keypoint> const& features, std::vector<bool> const& mask) {
    return CreateIndicesSpatialGrid<2, float>(
            cell_size, features, [](auto const& f) { return ConvertPoint(f); }, mask);
}
SpatialGridVo CreateSpatialGridVo(
        std::size_t const cell_size,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool> const& mask) {
    auto const f = [&features](auto const& m) { return ConvertPoint(features[m.index_b]); };
    return CreateIndicesSpatialGrid<2, float>(cell_size, matches, f, mask);
}

bool ValidateGlobalFlow(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::size_t const image_width,
        std::size_t const image_height,
        float const average_flow_threshold,
        std::size_t const grid_size) {
    std::size_t const grid_cell_size = std::ceil(static_cast<float>(std::min(image_width, image_height)) / grid_size);
    SpatialGridVo const grid = CreateSpatialGridVo(grid_cell_size, features, matches);

    float sum = 0;
    std::size_t count = 0;
    for (std::vector<std::size_t> const& data_bin : grid.GetBinnedData()) {
        if (!data_bin.empty()) {
            sum += detail::CalcFlowAverage(prev_features, features, matches, data_bin);
            ++count;
        }
    }
    return sum / count > average_flow_threshold;
}

double ComputeRelativeFactor(
        MatchVector const& matches_a,
        MatchVector const& matches_b,
        std::vector<Eigen::Vector3d> const& objects_a,
        std::vector<Eigen::Vector3d> const& objects_b) {
    assert(matches_a.size() == objects_a.size());
    assert(matches_b.size() == objects_b.size());

    // Find common matches (where index_a.index_b == index_b.index_a)
    std::vector<std::size_t> indices_a, indices_b;
    FindLinkedMatches(matches_a, matches_b, &indices_a, &indices_b);

    // At least to feature should be available, otherwise no distance can be calculated
    if (indices_a.size() < 2) {
        LOG(WARNING) << "The relative scale factor for the motion cannot be determined as there are not enough (less "
                        "than 2) feature matches between the last 3 key frames.";
        return 1.;
    }

    // Calculate the scaling factor for some 3d distances between features
    std::size_t const random_samples = indices_a.size();
    std::vector<double> factors;
    factors.reserve(random_samples);
    for (auto const& pair : detail::GetRandomPairs(0, indices_a.size(), random_samples)) {
        double distance_a = (objects_a[indices_a[pair.first]] - objects_a[indices_a[pair.second]]).norm();
        double distance_b = (objects_b[indices_b[pair.first]] - objects_b[indices_b[pair.second]]).norm();
        factors.push_back(distance_a / distance_b);
    }

    // Determine the relative scale by taking the median factor to minimize the influence of outliers
    std::sort(factors.begin(), factors.end());

    return factors[random_samples / 2];
}

bool DoBundleAdjustment(
        Eigen::Matrix3d const& K,
        std::vector<KeypointVector> const& keypoint_vectors,
        std::vector<Isometry3qd>* p_poses,
        std::vector<Eigen::Vector3d>* p_objects) {
    // Check in BuildVoBaProblem
    ceres::Problem problem;
    BuildVoBaProblem(K, keypoint_vectors, p_poses, p_objects, &problem);
    return SolveVoBaProblem(&problem);
}

}  // namespace nie
