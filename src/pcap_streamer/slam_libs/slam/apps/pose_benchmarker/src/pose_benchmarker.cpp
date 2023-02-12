/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "benchmark_poses.hpp"

DEFINE_string(reference_file, "", "Filepath to input reference / ground truth .pose file.");
DEFINE_string(benchmark_file, "", "Filepath to input .pose file to be benchmarked.");
DEFINE_string(out_file_prefix, "", "Filepath with prefix for output files.");

DEFINE_validator(reference_file, nie::ValidateIsFile);
DEFINE_validator(benchmark_file, nie::ValidateIsFile);
DEFINE_validator(out_file_prefix, nie::ValidateParentDirExists);

/// This app performs the following operations:
///     - read pose files
///     - interpolate reference poses to be compared to the benchmark poses
///     - find differences between benchmark poses and the interpolated reference poses
///     - write differences in position and orientation to csv file

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Read input files
    LOG(INFO) << "Reading reference pose file: " << FLAGS_reference_file;
    nie::io::PoseCollection pose_collection_reference =
            nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_reference_file);
    LOG(INFO) << "Read " << pose_collection_reference.poses.size() << " PoseRecords.";
    LOG(INFO) << "Reading benchmark pose file: " << FLAGS_benchmark_file;
    nie::io::PoseCollection pose_collection_benchmark =
            nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_benchmark_file);
    LOG(INFO) << "Read " << pose_collection_benchmark.poses.size() << " PoseRecords.";

    // Validate pose collections:
    // * If they use the same coordinate system
    // * If they have an overlap in time range
    nie::ValidatePoseCollectionCompatibility(pose_collection_reference, pose_collection_benchmark);

    // Generate output filenames
    std::string filename_matched{FLAGS_out_file_prefix + "_matches.pose"};
    std::string filename_csv{FLAGS_out_file_prefix + "_pose_difference.csv"};

    // Start actual processing
    LOG(INFO) << "Finding matching poses between reference and benchmark...";
    auto const matched_poses = nie::MatchReferenceCollection(pose_collection_reference, pose_collection_benchmark);

    LOG(INFO) << "Writing matched poses to file: " << filename_matched;
    nie::WriteMatchedPosesAsCollection(matched_poses, pose_collection_reference.header, filename_matched);

    LOG(INFO) << "Finding pose differences...";
    auto const pose_deltas = nie::FindPoseDeltas(matched_poses);

    // Open csv file and write header to file
    LOG(INFO) << "Writing pose differences to file: " << filename_csv;
    nie::WritePoseDeltasToCsv(pose_deltas, filename_csv);

    // TODO: Visualize differences
}
