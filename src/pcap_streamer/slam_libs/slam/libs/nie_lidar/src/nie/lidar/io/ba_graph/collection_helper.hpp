/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/formats/ba_graph/bbox_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <unordered_map>

#include "nie/lidar/geometry/pose_bbox.hpp"

namespace nie {

namespace io {

std::unordered_map<PoseId, PoseBbox> CreatePoseBboxMap(
        std::vector<PoseRecord> const& poses, std::vector<BboxRecord> const& boxes);

void CreatePoseBboxMap(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::BboxRecord> const& boxes,
        std::unordered_map<nie::io::PoseId, std::size_t>* pose_id_to_bound_index_map,
        std::vector<nie::PoseBbox>* bounds);

// TODO(jbr): Could be promoted to nie_formats.
Eigen::Matrix<double, 6, 6> InformationMatrixFromIsometry(
        nie::Isometry3qd const& ei, double const w, double const delta_percentage);

nie::io::PoseEdgeRecord MakeEdgeRecord(
        nie::io::PoseId const id0,
        nie::io::PoseId const id1,
        nie::Isometry3qd const& ei,
        double const w,
        double const delta_percentage);

// Distance treshold that may be used for determining which tracks are disconnected.
double GetMedianDistanceThreshold(PoseCollection const& collection);

}  // namespace io

}  // namespace nie
