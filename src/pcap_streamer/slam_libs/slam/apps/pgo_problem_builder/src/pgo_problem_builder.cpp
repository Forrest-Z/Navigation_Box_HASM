/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <unordered_set>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>

#include "pgo_problem_helper.hpp"

DEFINE_string(in_file_gps_pose, "", "The input gps pose file.");
DEFINE_string(in_file_lo_pose, "", "The input loam pose file.");
DEFINE_string(in_file_pose_ids_csv, "", "The vertex ids that are eligible to have edges attached to them.");
DEFINE_string(out_file_pgo_pose, "", "The output pose file.");

DEFINE_validator(in_file_gps_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_lo_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_ids_csv, nie::ValidateIsFile);
DEFINE_validator(out_file_pgo_pose, nie::ValidateParentDirExists);

std::unordered_set<nie::io::PoseId> ReadNodes(std::string const& filepath) {
    // Read csv file with ids
    nie::CsvRecorderSafe<nie::io::PoseId> recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({filepath}, "id", &recorder);
    auto const records = recorder.ConvertToRecord();
    // Loop over rows and extract ids
    std::unordered_set<nie::io::PoseId> nodes;
    nodes.reserve(records.NumRows());
    for (std::size_t i = 0; i < records.NumRows(); ++i) {
        nodes.insert(std::get<0>(records.GetRow(i)));
    }
    return nodes;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading gps pose file from: " << FLAGS_in_file_gps_pose;
    auto const gps_poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_gps_pose);

    LOG(INFO) << "Reading loam pose file from: " << FLAGS_in_file_lo_pose;
    auto const lo_poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_lo_pose);

    auto const eligible_pose_ids = ReadNodes(FLAGS_in_file_pose_ids_csv);

    CHECK(gps_poses.header.HasTimestampPerRecord()) << "Expecting timestamps for the input gps poses.";
    CHECK(lo_poses.header.HasTimestampPerRecord()) << "Expecting timestamps for the input loam poses.";

    LOG(INFO) << "Establishing PGO constraints for the input poses.";
    nie::io::PoseCollection pgo_poses;
    BuildPgoConstraints(gps_poses, lo_poses, eligible_pose_ids, &pgo_poses);

    nie::io::Write(pgo_poses, FLAGS_out_file_pgo_pose);

    return 0;
}
