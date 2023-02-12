/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "transform.hpp"

#include <nie/core/geometry/interpolation.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

namespace {

// Assumes that the poses are sorted on timestamp
void Check(nie::io::PoseCollection const& pose_collection, nie::Timestamp_ns const& time) {
    DCHECK(std::is_sorted(
            pose_collection.poses.cbegin(),
            pose_collection.poses.cend(),
            [](nie::io::PoseRecord const& a, nie::io::PoseRecord const& b) { return a.timestamp < b.timestamp; }))
            << "Pose collection should have poses sorted according to their timestamp.";

    auto const& poses = pose_collection.poses;
    CHECK(time >= poses.begin()->timestamp && time <= poses.rbegin()->timestamp)
            << "The timestamp of the first relative pose is not in timestamp range of the absolute poses.";
}

}  // namespace

nie::io::PoseCollection Transform(nie::io::PoseCollection const& pose_gps, nie::io::PoseCollection const& pose_odom) {
    // In determining the overall transformation, given:
    //  - transformation from the moving lidar device origin to lidar device origin at start of loam trajectory, T_loam
    //  - transformation from the lidar device origin at start of loam trajectory to the chosen reference point in loam
    //    trajectory, T_ref_loam (constant)
    //  - transformation from the gps origin at reference point of loam trajectory to the world, T_world_gps_ref
    //  (constant)
    // applying T_world_gps_ref * T_ref_loam transforms the moving lidar origin to world coordinates, named T_before:
    //    T_world_gps = T_world_gps_ref * T_ref_loam * T_loam
    //                = T_before                     * T_loam
    nie::io::PoseRecord const& ref_pose = pose_odom.poses[std::floor(pose_odom.poses.size() / 2)];
    nie::Isometry3qd const& T_loam_ref = ref_pose.isometry;

    Check(pose_gps, ref_pose.timestamp);
    nie::Isometry3qd T_world_gps_ref;
    nie::io::InterpolateIsometry(pose_gps.poses, ref_pose.timestamp, &T_world_gps_ref);

    nie::Isometry3qd const T_before = T_world_gps_ref * T_loam_ref.Inversed();

    // Create new transformed poses
    nie::io::PoseCollection result = pose_odom;
    for (auto& p : result.poses) {
        p.isometry = T_before * p.isometry;
    }

    // Just recalculate the edges based on the new poses
    auto const map_pose = nie::io::CreateRecordMap(result.poses.cbegin(), result.poses.cend());
    for (auto& e : result.edges) {
        e.isometry = map_pose.at(e.id_begin).get().isometry.Inversed() * map_pose.at(e.id_end).get().isometry;
    }

    return result;
}
