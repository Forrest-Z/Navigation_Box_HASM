/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>

#include "filter_loops.hpp"

DEFINE_string(in_file_loops, "", "Filepath to the loops .pose file.");
DEFINE_string(in_file_bbox_pose, "", "Filepath to input oriented bbox .pose file in the same system as the trace.");
DEFINE_string(in_file_bbox_bbox, "", "Filepath to input oriented bbox .bbox file.");
DEFINE_string(in_file_bbox_iref, "", "Filepath to input oriented bbox .iref file.");
DEFINE_string(in_file_trace_pose, "", "Filepath to input trace .pose file.");
DEFINE_string(in_file_trace_iref, "", "Filepath to input trace .iref file.");
DEFINE_string(out_file_loops, "", "Filepath to output .pose file.");

DEFINE_double(intersection_area_threshold, 20000, "Minimum oriented bounding box overlap for loops.");
DEFINE_double(intersection_ratio_threshold, 0.1, "Minimum oriented bounding box ratio for loops.");
DEFINE_double(minimum_trace_length, 5.0, "Minimum trace length corresponding to each bbox.");
DEFINE_double(
        maximum_trace_cov_score,
        0.1,
        "Maximum trace position covariance score to be considered as reliable. A 0.0 value disables this filter.");

DEFINE_validator(in_file_loops, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox_iref, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_iref, nie::ValidateIsFile);
DEFINE_validator(out_file_loops, nie::ValidateParentDirExists);
DEFINE_validator(intersection_area_threshold, nie::ValidateLargerThanZero);
DEFINE_validator(intersection_ratio_threshold, nie::ValidateLargerThanZero);
DEFINE_validator(minimum_trace_length, nie::ValidateLargerThanZero);
DEFINE_validator(maximum_trace_cov_score, nie::ValidateLargerOrEqualToZero);

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading loops file: " << FLAGS_in_file_loops;
    auto loops_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_loops);
    VLOG(1) << "Read number of candidates: " << loops_pose.edges.size();

    LOG(INFO) << "Reading bbox pose file: " << FLAGS_in_file_bbox_pose;
    auto const bbox_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_bbox_pose);

    LOG(INFO) << "Reading bbox bbox file: " << FLAGS_in_file_bbox_bbox;
    auto const bbox_bbox = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_bbox_bbox);

    LOG(INFO) << "Reading bbox iref file: " << FLAGS_in_file_bbox_iref;
    auto const bbox_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_bbox_iref);

    LOG(INFO) << "Reading trace pose file: " << FLAGS_in_file_trace_pose;
    auto const trace_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_pose);

    LOG(INFO) << "Reading trace iref file: " << FLAGS_in_file_trace_iref;
    auto const trace_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_trace_iref);

    LOG(INFO) << "Filtering the loop candidates...";

    auto const loops_filtered_pose = nie::FilterLoops(
            loops_pose,
            bbox_pose,
            bbox_bbox,
            bbox_iref,
            trace_pose,
            trace_iref,
            FLAGS_intersection_area_threshold,
            FLAGS_intersection_ratio_threshold,
            FLAGS_minimum_trace_length,
            FLAGS_maximum_trace_cov_score);

    VLOG(1) << "Reduced the number of candidates from " << loops_pose.edges.size() << " to "
            << loops_filtered_pose.edges.size();

    LOG(INFO) << "Writing filtered loops to .pose file: " << FLAGS_out_file_loops;
    nie::io::Write(loops_filtered_pose, FLAGS_out_file_loops);
}
