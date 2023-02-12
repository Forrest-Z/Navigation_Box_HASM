/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>

#include "transform.hpp"

DEFINE_string(in_file_pose_gps, "", "The input pose file with the data to determine the offset.");
DEFINE_string(in_file_pose_odom_rel, "", "The input pose file for which the poses are updated.");
DEFINE_string(out_file_pose_odom_abs, "", "The output pose file with the updated poses.");

DEFINE_validator(in_file_pose_gps, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_odom_rel, nie::ValidateIsFile);
DEFINE_validator(out_file_pose_odom_abs, nie::ValidateParentDirExists);

// The pose collection containing the relative poses / loam trajectory will be copied to the output collection, with the
// only modification that all poses will be transformed. This final poses represent the transformation from the local
// GPS origin to the world absolute coordinates based on the lidar odometry. This keeps the relative loam edges, but
// applies one transformation to all poses based on the offset based on the middle relative pose.

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

    auto const pose_abs = ReadFile(FLAGS_in_file_pose_gps, "gps");
    auto const pose_rel = ReadFile(FLAGS_in_file_pose_odom_rel, "odom_rel");

    LOG(INFO) << "Writing odom_abs pose file to: " << FLAGS_out_file_pose_odom_abs;
    nie::io::Write(Transform(pose_abs, pose_rel), FLAGS_out_file_pose_odom_abs);

    return 0;
}
