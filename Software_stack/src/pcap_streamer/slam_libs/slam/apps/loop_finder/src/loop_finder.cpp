/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>

#include "find_loops.hpp"

DEFINE_string(in_file, "", "Filepath to input .pose file.");
DEFINE_string(out_file, "", "Filepath to output .pose file.");
DEFINE_double(max_distance, 0.0, "Maximum distance allowed between two poses for them to be considered a loop closure");
DEFINE_double(
        min_time, 0, "Minimum time difference [seconds] required between two poses to be considered for loop closure.");
// TODO: add parameters for specifying threshold for a loop.
//  For two poses to be considered a loop closing the follow thresholds must be satisfied:
//  - minimum travelled distance
//  - minimum number of poses

DEFINE_validator(in_file, nie::ValidateIsFile);
DEFINE_validator(out_file, nie::ValidateStringNotEmpty);
DEFINE_validator(max_distance, nie::ValidateLargerThanZero);
DEFINE_validator(min_time, nie::ValidateLargerThanZero);

/// This app performs the following operations:
///     - read pose file
///     - find edges between pair of poses that close a loop
///     - write loop closure candidates to csv file

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading pose file: " << FLAGS_in_file;
    nie::io::PoseCollection pose_collection = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file);
    LOG(INFO) << "Read " << pose_collection.poses.size() << " PoseRecords.";

    VLOG(1) << "First PoseRecord timestamp: " << pose_collection.poses.cbegin()->timestamp;
    VLOG(1) << "Last PoseRecord timestamp: " << pose_collection.poses.crbegin()->timestamp;

    LOG(INFO) << "Finding loop closures...";
    // Zero initialize (meaning that it won't write information matrices or timestamps).
    nie::io::PoseCollection collection{};
    collection.header = pose_collection.header;
    collection.edges = nie::FindLoops(
            pose_collection.poses,
            FLAGS_max_distance,
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(FLAGS_min_time)));
    LOG(INFO) << "Found " << collection.edges.size() << " loop closures.";
    nie::io::Write(collection, FLAGS_out_file);
}
