/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/collection_writer.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

/// This app reads a pose file containing an odometry trajectory. The poses are looked up in the gps and a comparison is
/// done. When the poses are too different, then the information matrix of the odometry is scaled down (standard
/// deviation is increased).

DEFINE_string(in_file_pose_odom, "", "Filepath to input .pose file with odometry trajectory to reweight.");
DEFINE_string(in_file_pose_gps, "", "Filepath to input .pose file with the gps trajectory.");
DEFINE_string(out_file_pose, "", "Filepath to output .pose file.");

DEFINE_validator(in_file_pose_odom, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_gps, nie::ValidateIsFile);

void ReweightEdges(std::vector<nie::io::PoseRecord> const& gps_poses, nie::io::PoseCollection* p_odom_pose_collection) {
    auto const& odom_poses = p_odom_pose_collection->poses;
    auto& odom_edges = p_odom_pose_collection->edges;

    CHECK(odom_poses.front().timestamp >= gps_poses.front().timestamp)
            << "Start of trajectory " << odom_poses.front().timestamp << " is before the start of trace "
            << gps_poses.front().timestamp;
    CHECK(odom_poses.back().timestamp <= gps_poses.back().timestamp)
            << "End of trajectory " << odom_poses.back().timestamp << " is after the end of trace "
            << gps_poses.back().timestamp;

    // Create a map to lookup the edge pose id to get the actual pose records.
    auto const odom_pose_map = nie::io::CreateRecordMap(odom_poses.cbegin(), odom_poses.cend());
    auto iterators = std::make_pair(gps_poses.cbegin(), gps_poses.cend());

    for (auto& odom_edge : odom_edges) {
        auto const& timestamp_begin = odom_pose_map.at(odom_edge.id_begin).get().timestamp;
        auto const& timestamp_end = odom_pose_map.at(odom_edge.id_end).get().timestamp;

        nie::Isometry3qd pose_begin, pose_end;
        iterators = nie::io::InterpolateIsometry(iterators.first, gps_poses.cend(), timestamp_begin, &pose_begin);
        iterators = nie::io::InterpolateIsometry(iterators.first, gps_poses.cend(), timestamp_end, &pose_end);
        auto const gps_edge_isom = pose_begin.TransformInverseLeft(pose_end);

        double constexpr sigma = 3.;
        double const factor = 1. + sigma * odom_edge.isometry.TransformInverseLeft(gps_edge_isom).translation().norm() /
                                           gps_edge_isom.translation().norm();
        odom_edge.information /= std::pow(factor, 2);
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading odometry pose file: " << FLAGS_in_file_pose_odom;
    auto const odom_poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_odom);

    LOG(INFO) << "Reading gps pose file: " << FLAGS_in_file_pose_odom;
    auto const gps_poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_gps);

    nie::io::PoseCollection reweighted_poses = odom_poses;
    ReweightEdges(gps_poses.poses, &reweighted_poses);

    LOG(INFO) << "Writing reweighted pose file: " << FLAGS_out_file_pose;
    nie::io::Write(reweighted_poses, FLAGS_out_file_pose);
}
