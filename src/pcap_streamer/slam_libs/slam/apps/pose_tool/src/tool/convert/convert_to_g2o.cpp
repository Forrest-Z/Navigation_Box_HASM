/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_to_g2o.hpp"

#include <gflags/gflags.h>
#include <nie/formats/g2o_writer.hpp>

#include "tool/io.hpp"

DEFINE_bool(with_origin, false, "Whether an origin should be output to store the pose covariances.");
DEFINE_int32(
        origin_id,
        -1001,
        "The origin used in an edge corresponding to the pose covariance. Default is value is -1001.");

using Matrix6d = Eigen::Matrix<double, 6, 6>;

template <typename Id>
nie::io::G2oEdgeSe3Quat CreateG2oEdge(
        Id const& id_begin, Id const& id_end, nie::Isometry3qd const& isom, Matrix6d const& info) {
    nie::io::G2oEdgeSe3Quat g;
    g.id_begin = static_cast<nie::io::PoseId>(id_begin);
    g.id_end = static_cast<nie::io::PoseId>(id_end);
    g.t_be = isom;
    g.information = info;
    return g;
}

void ConvertToG2o() {
    InPathExistsOrFatal(nie::io::graph::Extension<nie::io::PoseCollection>());
    CheckOutPathsLocationsOrFatal();

    boost::filesystem::path path;
    GetAndCheckOutPathsForExtensionOrFatal(".g2o", &path);
    std::string const out_file_name = path.string();

    nie::io::PoseCollection pose_coll;
    ReadData(&pose_coll);

    nie::io::MapOfPoses poses;
    nie::io::VectorOfConstraints constraints;

    bool const pose_info_available = pose_coll.header.HasAnyPoseInformation();
    bool const pose_info_available_per_record = pose_coll.header.HasPoseInformationPerRecord();
    bool const edge_info_available_per_record = pose_coll.header.HasEdgeInformationPerRecord();

    Matrix6d const& default_pose_info =
            pose_coll.header.HasPoseInformation() ? pose_coll.header.pose_information : Matrix6d::Identity();
    Matrix6d const& default_edge_info =
            pose_coll.header.HasEdgeInformation() ? pose_coll.header.edge_information : Matrix6d::Identity();

    // Add all poses
    // As the g2o format can not contain information for a pose, still this behaviour is mimicked by:
    //   * creating some fixed origin, and
    //   * creating an edge between every pose and the origin with the pose information available
    if (FLAGS_with_origin && pose_info_available) {
        poses.emplace(FLAGS_origin_id, nie::io::G2oVertexSe3Quat{FLAGS_origin_id, nie::Isometry3qd::Identity()});
    }
    for (nie::io::PoseRecord const& pose : pose_coll.poses) {
        poses.emplace(pose.id, nie::io::G2oVertexSe3Quat{pose.id, pose.isometry});
        if (FLAGS_with_origin && pose_info_available) {
            constraints.push_back(CreateG2oEdge(
                    FLAGS_origin_id,
                    pose.id,
                    pose.isometry,
                    pose_info_available_per_record ? pose.information : default_pose_info));
        }
    }

    // Add all poses
    for (nie::io::PoseEdgeRecord const& edge : pose_coll.edges) {
        constraints.push_back(CreateG2oEdge(
                edge.id_begin,
                edge.id_end,
                edge.isometry,
                edge_info_available_per_record ? edge.information : default_edge_info));
    }

    CHECK(WriteG2oFile(out_file_name, poses, constraints)) << "Error writing to file: " << out_file_name;
    LOG(INFO) << "g2o file written.";
}
