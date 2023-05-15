/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>

#include "ground_truth_compare.hpp"
#include "io.hpp"

// The first entries for the loops files are taken to be the reference files. The rest are considered test files and
// will be checked against the reference one. When the ground truth (for the reference data) is supplied, then the
// transformations of the test files will be compared with the reference one and reported as being the same or different
// based on the thresholds given.

// clang-format off
DEFINE_string(in_files_unfiltered_loops, "", "Filepath to the unfiltered loops .pose files.");
DEFINE_string(in_files_filtered_loops, "", "Filepath to the filtered loops .pose files.");
DEFINE_string(in_files_closure_loops, "", "Filepath to the closure loops .pose files.");
DEFINE_string(in_file_ground_truth, "",
        "Filepath to the .csv file having the annotation of which edges are correct or not. [optional]");

DEFINE_double(translation_threshold, 0.1,
        "The threshold for the translation in meters to mark a transformation as different.");
DEFINE_double(rotation_threshold, 0.1,
        "The threshold for the overall rotation angle in degrees to mark a transformation as different.");
// clang-format on

DEFINE_validator(translation_threshold, nie::ValidateLargerOrEqualToZero);
DEFINE_validator(rotation_threshold, nie::ValidateLargerOrEqualToZero);

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    auto const loops_data =
            ReadPoseFiles(FLAGS_in_files_unfiltered_loops, FLAGS_in_files_filtered_loops, FLAGS_in_files_closure_loops);

    if (!FLAGS_in_file_ground_truth.empty()) {
        CHECK(nie::ValidateIsFile("in_file_ground_truth", FLAGS_in_file_ground_truth));
    }
    auto const ground_truth =
            FLAGS_in_file_ground_truth.empty() ? ClosureGroundTruthMap{} : ReadGroundTruth(FLAGS_in_file_ground_truth);

    Statistics stats{};

    // Add the basic edge counts to the statistics
    for (PoseSet const& data : loops_data) {
        stats.original_matches.push_back(data.unfiltered.edges.size());
        stats.filtered_matches.push_back(data.filtered.edges.size());
        stats.succeeded_matches.push_back(data.closure.edges.size());
    }

    // Compare the resulting edges with the ground truth
    if (!ground_truth.empty()) {
        AddGroundTruthStatistics(
                loops_data, ground_truth, FLAGS_translation_threshold, FLAGS_rotation_threshold, &stats);
    }

    stats.Print(!ground_truth.empty());
}
