/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/lidar/cloud.hpp>

#include "helper_cloud.hpp"

namespace nie {

template <typename PointT>
void PointDensity(
    nie::Cloud<PointT> const& cloud_a,
    nie::Cloud<PointT> const& cloud_b,
    std::vector<std::pair<int, int>>* neighbors,
    double radius = 10.) {
    int const kAmountNearestNeighbors = 10;

    auto cloud_a_tree = GetKdTree(cloud_a, true);
    auto cloud_b_tree = GetKdTree(cloud_b, true);

    // Round-trip search:
    //  - for every point in a, look for the neighbors in b
    //  - for every neighbor in b, look for the neighbors in a again
    //  - stop when a neighbor in a is the same as the original point in a
    std::vector<float> tmp_distances;
    for (std::size_t index_a = 0; index_a < cloud_a.point_cloud().size(); ++index_a) {
        std::vector<int> indices_b;
        cloud_b_tree->radiusSearch(
            cloud_a.point_cloud().points[index_a], radius, indices_b, tmp_distances, kAmountNearestNeighbors);

        for (int index_b : indices_b) {
            std::vector<int> indices_a;
            cloud_a_tree->radiusSearch(
                cloud_b.point_cloud().points[index_b], radius, indices_a, tmp_distances, kAmountNearestNeighbors);

            if (std::find(indices_a.begin(), indices_a.end(), index_a) != indices_a.end()) {
                neighbors->emplace_back(index_a, index_b);
                break;
            }
        }
    }
}

}  // namespace nie
