/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/spatial_grid.hpp>

#include "feature/descriptor_extractor.hpp"
#include "feature/detector.hpp"
#include "feature/match_filter.hpp"
#include "feature/matcher.hpp"

namespace nie {

// Convenience function to create spatial grid for features and matches (taking the current feature of the matches)
using SpatialGridVo = SpatialGrid<std::size_t, 2, float>;
SpatialGridVo CreateSpatialGridVo(
        std::size_t const cell_size, std::vector<Keypoint> const& features, std::vector<bool> const& mask = {});
SpatialGridVo CreateSpatialGridVo(
        std::size_t const cell_size,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool> const& mask = {});

bool ValidateGlobalFlow(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::size_t const image_width,
        std::size_t const image_height,
        float const average_flow_threshold,
        std::size_t const grid_size = 5);

template <typename Derived, typename Rotation>
bool EstimateMotion(
        KeypointVector const& features_a,
        KeypointVector const& features_b,
        cv::Matx33d const& K,
        MatchVector* p_matches,
        Isometry3Base<Derived, Rotation>* p_motion);

template <typename Isometry>
void TriangulateFeatures(
        std::vector<Isometry> const& poses,
        std::vector<KeypointVector> const& features,
        std::vector<std::vector<std::size_t>> const& matched_features,
        Eigen::Matrix3d const& K,
        std::vector<Eigen::Vector3d>* p_objects);

/// Calculate the relative scaling between the current (b) and previous (a) objects
/// The following steps are performed:
///   * Based on the matching information, common objects in a and b are identified.
///   * For random pairs in the objects in a the distance between those objects are calculated.
///   * Similar to previous step, distances between objects in b are calculated based on the same random pairs
///   * For every pair, the ratio of distance in a over b is calculated.
///   * The returned result is the median value of all factors.
///
// TODO (MvB): Should only needs the 3d points and maybe the correspondences. Now the correspondences are calculated
//             internally.
double ComputeRelativeFactor(
        MatchVector const& matches_a,
        MatchVector const& matches_b,
        std::vector<Eigen::Vector3d> const& objects_a,
        std::vector<Eigen::Vector3d> const& objects_b);

// The keypoint indices have a value of -1 when it is not present in that frame...
bool DoBundleAdjustment(
        Eigen::Matrix3d const& K,
        std::vector<KeypointVector> const& keypoint_vectors,
        std::vector<Isometry3qd>* p_poses,
        std::vector<Eigen::Vector3d>* p_objects);

}  // namespace nie

#include "visual_odometry.inl"
