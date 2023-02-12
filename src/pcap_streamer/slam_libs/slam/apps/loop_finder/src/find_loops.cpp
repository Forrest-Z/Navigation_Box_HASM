/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "find_loops.hpp"

#include <glog/logging.h>

#include <nie/core/time.hpp>
#include <nie/cv/kd_tree.hpp>
#include <nie/formats/ba_graph/pose_record_kd_tree_adapter.hpp>

namespace nie {

//  Convenience typedef for 2D KdTrees of PoseRecords
using KdTreePoseRecord2D = KdTree<double, PoseRecordKdTreeAdapter, 2>;

std::vector<io::PoseEdgeRecord> FindLoops(
        std::vector<io::PoseRecord> const& poses,
        double const max_distance,
        std::chrono::nanoseconds const min_time_delta,
        std::size_t const max_leaf_size) {
    DLOG(INFO) << "max_distance = " << max_distance;
    DLOG(INFO) << "min_time_delta = " << min_time_delta.count();

    double const max_distance_sqr = max_distance * max_distance;

    PoseRecordKdTreeAdapter adapter(poses);

    VLOG(1) << "Building KD tree...";
    KdTreePoseRecord2D kd_tree{adapter, max_leaf_size};
    VLOG(1) << "Building KD tree done.";

    std::vector<io::PoseEdgeRecord> result{};

    VLOG(1) << "Iterating PoseRecords...";
    std::vector<KdTreePoseRecord2D::MatchType> matches;
    for (auto const& pose1 : poses) {
        size_t matches_count = kd_tree.RadiusSearch(pose1, max_distance_sqr, &matches);

        VLOG(12) << "Radius search for pose " << pose1.id << " found " << matches_count << " matches.";

        // The first one is always itself (because we use it to perform the search and the returned vector is sorted).
        for (std::size_t i = 1; i < matches_count; ++i) {
            auto const& pose2 = poses[matches[i].first];

            // skip if pose2 is not at least min_time_delta later than pose
            // As time_delta is signed, this actually also avoids duplicate matching a-b and b-a
            auto time_delta = pose2.timestamp - pose1.timestamp;
            if (time_delta < min_time_delta) {
                continue;
            }

            // We use the typical L2 metric (which is a squared number and efficient) and thus need to sqrt for a
            // print.
            VLOG(9) << "Found loop closure candidate: " << pose1.id << ", " << pose2.id << ", distance "
                    << std::sqrt(matches[i].second) << ", time delta " << time_delta.count();

            io::PoseEdgeRecord r;
            r.id_begin = pose1.id;
            r.id_end = pose2.id;
            r.isometry = pose1.isometry.TransformInverseLeft(pose2.isometry);
            r.category = io::PoseEdgeRecord::Category::kLoop;
            result.push_back(r);
        }
    }

    return result;
}

}  // namespace nie
