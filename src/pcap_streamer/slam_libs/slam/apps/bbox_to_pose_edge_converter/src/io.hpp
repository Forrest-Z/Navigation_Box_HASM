/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <unordered_map>
#include <unordered_set>

#include <nie/formats/ba_graph.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>

/// Reads the selected ids from a csv and puts them in an undordered_set.
std::unordered_set<nie::io::PoseId> ReadNodes(std::string const& filepath);

/// Function that return a map going from the bounding box pose id to the trace pose id's that are related to each other
/// via the iref path. Only the trace pose id's present in the linkable pose id's will be added.
std::unordered_map<nie::io::PoseId, std::vector<nie::io::PoseId>> GetTraceIdsByBoxId(
        std::unordered_set<nie::io::PoseId> const& linkable_pose_ids,
        nie::io::InfoRefCollection const& bbox_iref_coll,
        nie::io::InfoRefCollection const& trace_iref_coll);

void FilterPoseIds(
        nie::io::PoseEdgeRecord const& bbox_edge,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& aa_bbox_pose_map,
        std::unordered_map<nie::io::PoseId, nie::PoseBbox> const& oa_bounds_map,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& trace_pose_map,
        std::vector<nie::io::PoseId>* p_trace_b,
        std::vector<nie::io::PoseId>* p_trace_e);
