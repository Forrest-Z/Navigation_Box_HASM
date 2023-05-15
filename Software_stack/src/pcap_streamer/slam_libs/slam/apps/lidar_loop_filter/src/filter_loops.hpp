/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/formats/ba_graph/bbox_collection.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

nie::io::PoseCollection FilterLoops(
        nie::io::PoseCollection const& bbox_loops,
        nie::io::PoseCollection const& bbox_pose,
        nie::io::BboxCollection const& bbox_bbox,
        nie::io::InfoRefCollection const& bbox_iref,
        nie::io::PoseCollection const& trajectory_pose,
        nie::io::InfoRefCollection const& trajectory_iref,
        double const intersection_area_threshold,
        double const intersection_ratio_threshold,
        double const minimum_trace_length,
        double const maximum_trace_cov_score);
}
