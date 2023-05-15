/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <chrono>
#include <vector>

#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

/// Detect loops in a sequence of poses.
///
/// \param poses  vector of io::PoseRecord
/// \param max_distance  maximum distance between two poses for them to be considered a loop closure.
/// \param min_time_delta  minimum time difference between two poses for them to be considered a loop closure.
/// \param max_leaf_size
/// \return  vector of io::PoseEdgeRecord, each edge is a loop closure.
std::vector<io::PoseEdgeRecord> FindLoops(
        std::vector<io::PoseRecord> const& poses,
        double const max_distance,
        std::chrono::nanoseconds const min_time_delta,
        std::size_t const max_leaf_size = 10);

}  // namespace nie
