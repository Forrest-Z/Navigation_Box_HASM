/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/geometry/rotation.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>

#include "generate_edges.hpp"
#include "io.hpp"

DEFINE_string(in_file_bbox_loops, "", "Filepath to the loops .pose file.");
DEFINE_string(in_file_aa_bbox_pose, "", "Filepath to input axis aligned bbox .pose file.");
DEFINE_string(in_file_aa_bbox_iref, "", "Filepath to input axis aligned bbox .iref file.");
DEFINE_string(in_file_oa_bbox_pose, "", "Filepath to input oriented axis bbox .pose file.");
DEFINE_string(in_file_oa_bbox_bbox, "", "Filepath to input oriented axis bbox .bbox file.");
DEFINE_string(in_file_trace_pose, "", "Filepath to input trajectory .pose file.");
DEFINE_string(in_file_trace_iref, "", "Filepath to input trajectory .iref file.");
DEFINE_string(in_file_trace_ids, "", "Filepath to the input trajectory filtered ids .csv file.");
DEFINE_string(out_file_trace_loops, "", "Filepath to output .loops file.");

// clang-format off
DEFINE_double(info_translation, 0.1,
        "The uncertainty in the translational part of the information matrix of the resulting edge [meter]. Default is "
        "10 cm.");
DEFINE_double(info_rotation, 0.1146,
        "The uncertainty in the rotational part of the information matrix of the resulting edges [degrees]. Default is "
        "0.1146 degrees.");
// clang-format on

DEFINE_validator(in_file_bbox_loops, nie::ValidateIsFile);
DEFINE_validator(in_file_aa_bbox_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_aa_bbox_iref, nie::ValidateIsFile);
DEFINE_validator(in_file_oa_bbox_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_oa_bbox_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_iref, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_ids, nie::ValidateIsFile);
DEFINE_validator(out_file_trace_loops, nie::ValidateParentDirExists);

DEFINE_validator(info_translation, nie::ValidateLargerOrEqualToZero);
DEFINE_validator(info_rotation, nie::ValidateLargerOrEqualToZero);

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    auto const linkable_pose_ids = ReadNodes(FLAGS_in_file_trace_ids);
    auto const aa_bbox_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_aa_bbox_iref);
    auto const trace_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_trace_iref);
    auto const trace_ids_by_bbox_id = GetTraceIdsByBoxId(linkable_pose_ids, aa_bbox_iref, trace_iref);

    auto const aa_bbox_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_aa_bbox_pose);
    auto const aa_bbox_pose_map = nie::io::CreateRecordMap(aa_bbox_pose.poses.begin(), aa_bbox_pose.poses.end());
    auto const trace_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_pose);
    auto const trace_pose_map = nie::io::CreateRecordMap(trace_pose.poses.begin(), trace_pose.poses.end());
    auto const bbox_loops = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_bbox_loops);
    auto const oa_bounds_map = nie::io::CreatePoseBboxMap(
            nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_oa_bbox_pose).poses,
            nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_oa_bbox_bbox).boxes);

    VLOG(3) << "Input bbox loops size: " << bbox_loops.edges.size() << std::endl;

    nie::io::PoseCollection trace_loops{};
    trace_loops.header = bbox_loops.header;

    // Add edge information
    auto const kStdDevPositionIcp = Eigen::Array3d::Constant(FLAGS_info_translation);
    auto const kStdDevRotation = Eigen::Array3d::Constant(nie::Deg2Rad<>(FLAGS_info_rotation / 2.));
    Eigen::Array<double, 6, 1> const kStdDevIcp = (Eigen::ArrayXd(6) << kStdDevPositionIcp, kStdDevRotation).finished();
    trace_loops.header.Set(nie::io::PoseCollection::Header::kHasEdgeInformation);
    trace_loops.header.edge_information = kStdDevIcp.pow(-2.).matrix().asDiagonal();

    for (auto const& bbox_edge : bbox_loops.edges) {
        // Copy
        auto trace_b = trace_ids_by_bbox_id.at(bbox_edge.id_begin);
        auto trace_e = trace_ids_by_bbox_id.at(bbox_edge.id_end);

        FilterPoseIds(bbox_edge, aa_bbox_pose_map, oa_bounds_map, trace_pose_map, &trace_b, &trace_e);

        GenerateEdgesForBboxes(bbox_edge, trace_b, trace_e, aa_bbox_pose_map, trace_pose_map, &trace_loops.edges);
    }

    VLOG(3) << "Output trace loops size: " << trace_loops.edges.size() << std::endl;

    LOG(INFO) << "Writing trace loops to .pose file: " << FLAGS_out_file_trace_loops;
    nie::io::Write(trace_loops, FLAGS_out_file_trace_loops);

    return 0;
}
