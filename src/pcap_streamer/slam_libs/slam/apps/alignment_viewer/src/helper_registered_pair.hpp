/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <unordered_map>
#include <unordered_set>

#include <Eigen/Eigen>
#include <boost/filesystem/path.hpp>

#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/lidar/cloud.hpp>

#include "registered_pair.hpp"

namespace nie {

// Return the translation and total angle (in degrees) of the ICP correction
std::pair<double, double> CalculateCorrection(RegisteredPair const& pair);

void SortRegisteredPairsByTranslationMagnitude(std::vector<RegisteredPair>* pairs);
void SortRegisteredPairsRandomly(std::vector<RegisteredPair>* pairs);

/// Create a vector of loop closure candidates.
///
/// \param bbox_pose     PoseCollection of bounding box poses.
/// \param bbox_iref     InfoRefCollection mapping bounding box pose ids to LAS file locations.
/// \param edges_pose    PoseCollection with the loop closure edges.
std::vector<RegisteredPair> GetLoopClosureCandidates(
        io::PoseCollection const& bbox_pose,
        io::InfoRefCollection const& bbox_iref,
        io::PoseCollection const& edges_pose);

}  // namespace nie
