/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "tool/tools.hpp"

DEFINE_bool(add_edges, false, "[Tool] Adds edges between poses read from a .pose file.");
DEFINE_bool(add_fixture, false, "[Tool] Adds a fixture for the first poses from a .pose file.");
DEFINE_bool(
        concat,
        false,
        "[Tool] Concatenate .pose and related files. Assumes no relationship between files and all files are "
        "concatenated in order of appearance.");
DEFINE_bool(convert_from, false, "[Tool] Convert from an input format to pose related files.");
DEFINE_bool(convert_to, false, "[Tool] Convert from pose related files to an output format.");
DEFINE_bool(filter, false, "[Tool] Filter the content of a pose file.");
DEFINE_bool(
        merge, false, "[Tool] Merges the contents of .pose files. Assumes identical ids represent the same vertex.");
DEFINE_bool(print, false, "[Tool] Print the content of pose related files.");
DEFINE_bool(resample, false, "[Tool] Resamples a pose file and generates a new one.");
DEFINE_bool(shift, false, "[Tool] Shifts the content of a pose file in time and/or position.");

// TODO(jbr): Later also pose by checking extension
// TODO(jbr): Rename to subsample_ids ?
DEFINE_bool(subsample, false, "[Tool] Subsamples the content of a pose file as ids in a csv file.");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    if (FLAGS_add_edges) {
        LOG(INFO) << "[add_edges] Add edges between poses.";
        AddEdges();
    } else if (FLAGS_add_fixture) {
        LOG(INFO) << "[add_fixture] Add a fixture to the first pose.";
        AddFixture();
    } else if (FLAGS_concat) {
        LOG(INFO) << "[concat] Concatenating various .pose related files.";
        Concat();
    } else if (FLAGS_convert_from) {
        LOG(INFO) << "[convert_from] Converting specified format to binary binary pose graph formats.";
        ConvertFrom();
    } else if (FLAGS_convert_to) {
        LOG(INFO) << "[convert_to] Converting binary pose graph files to the specified format.";
        ConvertTo();
    } else if (FLAGS_filter) {
        LOG(INFO) << "[filter] Filtering ids from .pose related files.";
        Filter();
    } else if (FLAGS_merge) {
        LOG(INFO) << "[merge] Merging the contents of .pose or .iref files.";
        Merge();
    } else if (FLAGS_print) {
        LOG(INFO) << "[print] Printing .pose related files in text format.";
        Print();
    } else if (FLAGS_resample) {
        LOG(INFO) << "[resample] Resampling .pose file to another pose file.";
        Resample();
    } else if (FLAGS_shift) {
        LOG(INFO) << "[shift] Shifting .pose file to another pose file.";
        Shift();
    } else if (FLAGS_subsample) {
        LOG(INFO) << "[subsample] Subsampling .pose file to csv with ids.";
        Subsample();
    } else {
        LOG(FATAL) << "Select one of the tool flags!";
    }

    return 0;
}
