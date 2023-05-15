/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "pgo_problem_helper.hpp"

#include <chrono>

#include <glog/logging.h>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

// TODO(jbr) generalize and streamline with what is in pose_collection.hpp
template <typename Iterator>
inline std::pair<Iterator const, Iterator const> ViableEdge(
    Iterator const& begin,
    Iterator const& end,
    nie::Timestamp_ns const& timestamp,
    std::chrono::nanoseconds const& delta_threshold,
    bool* viable) {
    auto iterators = std::equal_range(begin, end, timestamp);

    // If they are equal we found a range, but both point 1 past the searched element.
    // Else ... we found an exact time stamp and the second iterator points one past the element (perhaps end())

    if (iterators.first == iterators.second) {
        iterators.first--;
        *viable = (iterators.second->timestamp - iterators.first->timestamp) < delta_threshold;
    } else {
        iterators.second--;
        // Not that this is bad, but for now no edge when nothing happened
        *viable = false;
    }

    return iterators;
}

std::chrono::nanoseconds MedianTimestampDifference(std::vector<nie::io::PoseRecord> const& records) {
    std::vector<std::chrono::nanoseconds> deltas(records.size() - 1);
    for (std::size_t i = 1; i < records.size(); ++i) {
        deltas[i - 1] = records[i].timestamp - records[i - 1].timestamp;
    }

    std::sort(deltas.begin(), deltas.end());

    std::size_t index = deltas.size() / 2;
    if ((deltas.size() & 1u) == 1) {
        return deltas[index];
    } else {
        return (deltas[index] + deltas[index + 1]) / 2;
    }
}

void BuildPgoConstraints(
    nie::io::PoseCollection const& pose_collection_gps,
    nie::io::PoseCollection const& pose_collection_abs,
    std::unordered_set<nie::io::PoseId> const& eligible_pose_ids,
    nie::io::PoseCollection* pose_collection_pgo) {
    CHECK(pose_collection_gps.header.HasPoseInformationPerRecord())
        << "Can not build PGO problem without per-pose gps information.";
    CHECK(pose_collection_abs.header.HasEdgeInformationPerRecord())
        << "Can not build PGO problem without per-edge odom information.";

    auto median_delta = MedianTimestampDifference(pose_collection_gps.poses);
    auto median_delta_buffered = median_delta + median_delta / 4;

    LOG(INFO) << "Median delta time gps: " << std::chrono::duration<double>(median_delta).count() << std::endl;
    LOG(INFO) << "Median delta time threshold gps: " << std::chrono::duration<double>(median_delta_buffered).count()
              << std::endl;

    // Update the poses in the given pose collection
    nie::io::PoseCollection& result = *pose_collection_pgo;
    result = pose_collection_abs;
    result.header.Set(nie::io::PoseHeader::kHasPoseInformationPerRecord);

    // For every pose:
    //   - Create a "copy" of the pose as expected by the PGO problem structure.
    //   - Set the information of the LOAM vertex to zero.
    //   - Add an identity information edge between the original pose and its copy.
    auto new_pose_id = nie::io::GetMaxPoseId(result) + 1;
    result.poses.reserve(result.poses.size() * 2);
    result.edges.reserve(result.edges.size() + result.poses.size());

    // Insert new GPS poses with edges to the original loam poses
    std::size_t added_count = 0;
    auto iterators = std::make_pair(pose_collection_gps.poses.begin(), pose_collection_gps.poses.end());
    for (std::size_t i = 0; i < pose_collection_abs.poses.size(); ++i) {
        auto const& pose = pose_collection_abs.poses[i];

        bool viable;
        iterators = ViableEdge(
            iterators.first, pose_collection_gps.poses.end(), pose.timestamp, median_delta_buffered, &viable);

        if (viable && (eligible_pose_ids.count(pose.id) > 0)) {
            nie::io::PoseRecord pose_copy = pose;
            pose_copy.category = nie::io::PoseRecord::Category::kGps;
            nie::io::detail::InterpolateFromPoseRecords(
                *iterators.first, *iterators.second, pose_copy.timestamp, &pose_copy.isometry, &pose_copy.information);

            // The GPS vertex needs a fresh id.
            pose_copy.id = new_pose_id++;
            // Add GPS vertex
            result.poses.push_back(pose_copy);
            // Push edge that connects the interpolated relative vertex to the GPS
            // one, with identity Information.
            result.edges.push_back({pose.id,
                                    pose_copy.id,
                                    nie::io::PoseEdgeRecord::Category::kRelToAbs,
                                    nie::Isometry3qd::Identity(),
                                    Eigen::Matrix<double, 6, 6>::Identity()});

            added_count++;
        }

        // While the GPS vertex keeps the Information from the absolute position the odometry vertex has its information
        // set to zero.
        result.poses[i].information.setZero();
    }

    LOG(INFO) << "Added " << added_count << " edges for " << pose_collection_abs.poses.size() << " vertices.";
}
