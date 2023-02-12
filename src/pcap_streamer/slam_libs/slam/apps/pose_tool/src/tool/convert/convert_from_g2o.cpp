/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_from_g2o.hpp"

#include <nie/formats/g2o_reader.hpp>

#include "tool/io.hpp"

nie::io::PoseRecord G2oVertexToPose(nie::io::G2oVertexSe3Quat const& v) {
    nie::io::PoseRecord p{};
    p.id = static_cast<nie::io::PoseId>(v.id);
    p.isometry = v.v;
    // Don't use the device_id and time_stamp here
    p.device_id = 0;
    p.timestamp = nie::Timestamp_ns{};
    return p;
}

nie::io::PoseEdgeRecord G2oEdgeToPoseEdge(nie::io::G2oEdgeSe3Quat const& g) {
    nie::io::PoseEdgeRecord e{};
    e.id_begin = static_cast<nie::io::PoseId>(g.id_begin);
    e.id_end = static_cast<nie::io::PoseId>(g.id_end);
    e.isometry = g.t_be;
    e.information = g.information;
    return e;
}

void ConvertFromG2o() {
    InPathExistsOrFatal();
    boost::filesystem::path path;
    GetAndCheckOutPathsForExtensionOrFatal(nie::io::graph::Extension<nie::io::PoseCollection>(), &path);
    std::string const out_file_name = path.string();

    nie::io::MapOfPoses poses;
    nie::io::VectorOfConstraints constraints;
    CHECK(nie::io::ReadG2oFile(FLAGS_in_paths, &poses, &constraints)) << "Error reading from file: " << FLAGS_in_paths;

    LOG(INFO) << "Supported g2o tags:";
    LOG(INFO) << "- " << nie::io::G2oVertexSe3Quat::Tag();
    LOG(INFO) << "- " << nie::io::G2oEdgeSe3Quat::Tag();
    LOG(INFO) << "Note: The first vertex is considered the origin and will be fixed.";

    CHECK(!poses.empty()) << "g2o file has no vertices.";
    CHECK(!constraints.empty()) << "g2o file has no edges.";

    {
        nie::io::PoseCollection pc{};
        nie::io::SetNieAuthority(&pc.header);
        pc.poses.reserve(poses.size());
        pc.edges.reserve(constraints.size());

        auto const& origin_id = poses.begin()->first;

        std::unordered_map<decltype(nie::io::G2oVertexSe3Quat::id), std::size_t> id_to_index;
        std::size_t index = 0;
        // Omit the first origin vertex as it will be added later when required
        for (auto pose = ++poses.begin(); pose != poses.end(); ++pose, ++index) {
            nie::io::PoseRecord pose_record = G2oVertexToPose(pose->second);
            pc.poses.push_back(pose_record);
            id_to_index[pose->first] = index;
        }

        // Determine whether information is available for all poses
        bool const pose_information_available =
            std::count_if(constraints.begin(), constraints.end(), [&origin_id](auto const& c) {
                return c.id_begin == origin_id;
            }) == static_cast<int>(pc.poses.size());

        if (pose_information_available) {
            pc.header.flags = nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord |
                              nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord;
            LOG(INFO) << "Every vertex is attached to the origin. Vertices will have an information matrix.";
        } else {
            pc.header.flags = nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord;
            LOG(INFO) << "Not every vertex is attached to the origin. Vertices will have no information matrix.";

            // As all edges are added (including the one to the origin), the origin vertex must be added and fixed
            pc.poses.push_back(G2oVertexToPose(poses.begin()->second));
            pc.fixes.push_back({pc.poses.back().id});
        }

        for (auto const& constraint : constraints) {
            // Identity constraint because the first one is considered fixed.
            if (pose_information_available and constraint.id_begin == origin_id) {
                pc.poses[id_to_index.at(constraint.id_end)].information = constraint.information;
                // As a sanity check it would be possible to see if the edge is close to equal to the vertex.
                // However, this tool is not meant to validate the input.
            } else {
                pc.edges.push_back(G2oEdgeToPoseEdge(constraint));
            }
        }

        nie::io::Write(pc, out_file_name);
    }
}
