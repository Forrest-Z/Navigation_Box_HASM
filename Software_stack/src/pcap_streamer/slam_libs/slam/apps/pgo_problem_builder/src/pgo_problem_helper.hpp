/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <unordered_set>

#include <nie/formats/ba_graph.hpp>

/// The pose collection containing poses in absolute coordinates and odometry
/// edges will be updated to match the PGO problem structure. That means that
/// for each pose we append a copy of itself to the end of the collection as
/// well as a new edge (with identity information matrix) connecting the pose
/// to its copy, whilst maintaining the original edges intact.
void BuildPgoConstraints(
    nie::io::PoseCollection const& pose_collection_gps,
    nie::io::PoseCollection const& pose_collection_abs,
    std::unordered_set<nie::io::PoseId> const& eligible_pose_ids,
    nie::io::PoseCollection* pose_collection_pgo);
