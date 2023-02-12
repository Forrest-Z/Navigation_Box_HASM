/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>

#include "transform.hpp"

DEFINE_string(in_file_pose_gps, "", "The input gps pose file.");
DEFINE_string(in_file_pose_odom, "", "The input odometry pose file.");
DEFINE_string(in_file_pose_bbox_odom, "", "The input bbox pose file in the same system as the odometry pose file.");
DEFINE_string(out_file_pose_bbox_gps, "", "The output bbox pose file in the same system as the gps pose file.");

DEFINE_validator(in_file_pose_gps, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_odom, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_bbox_odom, nie::ValidateIsFile);
DEFINE_validator(out_file_pose_bbox_gps, nie::ValidateParentDirExists);

nie::io::PoseCollection ReadFile(std::string const& filename, std::string const& name) {
    LOG(INFO) << "Reading " << name << " file from: " << filename;
    auto const pose = nie::io::ReadCollection<nie::io::PoseCollection>(filename);
    CHECK(pose.header.HasTimestampPerRecord()) << "Expecting timestamps to be available.";
    VLOG(2) << "Number of " << name << " poses: " << pose.poses.size();
    return pose;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    auto pose_gps = ReadFile(FLAGS_in_file_pose_gps, "gps");
    auto pose_odom = ReadFile(FLAGS_in_file_pose_odom, "odom");
    auto pose_bbox_odom = ReadFile(FLAGS_in_file_pose_bbox_odom, "pose_bbox_odom");

    LOG(INFO) << "Writing bbox_gps pose file to: " << FLAGS_out_file_pose_bbox_gps;
    nie::io::Write(Transform(pose_gps, pose_odom, pose_bbox_odom), FLAGS_out_file_pose_bbox_gps);

    return 0;
}
