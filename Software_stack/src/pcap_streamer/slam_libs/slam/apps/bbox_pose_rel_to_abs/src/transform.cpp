/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "transform.hpp"

#include <nie/core/geometry/interpolation.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

void Check(nie::io::PoseCollection const& collection, nie::Timestamp_ns const& timestamp) {
    CHECK(timestamp >= collection.poses.begin()->timestamp && timestamp <= collection.poses.rbegin()->timestamp)
            << "The timestamp of the first relative pose is not in timestamp range of the absolute poses.";
}

nie::io::PoseCollection Transform(
        nie::io::PoseCollection const& pose_gps,
        nie::io::PoseCollection const& pose_odom,
        nie::io::PoseCollection const& pose_bbox_odom) {
    auto its_pose_abs = std::make_pair(pose_gps.poses.begin(), pose_gps.poses.end());
    auto its_pose_rel = std::make_pair(pose_odom.poses.begin(), pose_odom.poses.end());
    auto pose_bbox_gps = pose_bbox_odom;

    for (auto& p : pose_bbox_gps.poses) {
        // Variable p is the bounding box center of mass. It has a timestamp that is used to determine where to place it
        // in the world. For this timestamp we get the gps position and the loam position. Since these two (should)
        // represent the exact same location in space, we can use the transformation between them to update the bounding
        // box center of mass to the world frame.

        Check(pose_gps, p.timestamp);
        nie::Isometry3qd T_world_gps;
        its_pose_abs =
                nie::io::InterpolateIsometry(its_pose_abs.first, pose_gps.poses.end(), p.timestamp, &T_world_gps);

        Check(pose_odom, p.timestamp);
        nie::Isometry3qd T_odom_lidar;
        its_pose_rel =
                nie::io::InterpolateIsometry(its_pose_rel.first, pose_odom.poses.end(), p.timestamp, &T_odom_lidar);

        p.isometry = T_world_gps * T_odom_lidar.Inversed() * p.isometry;
    }

    return pose_bbox_gps;
}
