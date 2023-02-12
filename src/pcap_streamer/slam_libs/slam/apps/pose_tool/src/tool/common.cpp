/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "common.hpp"

#include <nie/core/gflags.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>

// clang-format off
DEFINE_double(max_edge_distance, -1.,
              "In case of add_edges and resample mode an interval between two poses with this maximum distance is ignored."
              " Default is -1 which means 3 times the median distance is taken.");
// clang-format on

double GetMaximumDistanceThreshold(nie::io::PoseCollection const& pose_collection) {
    double distance;
    if (FLAGS_max_edge_distance == -1.) {
        distance = nie::io::GetMedianDistanceThreshold(pose_collection);
    } else {
        CHECK(nie::ValidateLargerThanZero("max_edge_distance", FLAGS_max_edge_distance));
        distance = FLAGS_max_edge_distance;
    }
    return distance;
}
