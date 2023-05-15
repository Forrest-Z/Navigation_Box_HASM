/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/formats/ba_graph.hpp>

void GenerateEdgesForBboxes(
        nie::io::PoseEdgeRecord const& bbox_edge,
        std::vector<nie::io::PoseId> const& trace_b,
        std::vector<nie::io::PoseId> const& trace_e,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& aa_bbox_pose_map,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& trace_pose_map,
        std::vector<nie::io::PoseEdgeRecord>* trace_loops);
