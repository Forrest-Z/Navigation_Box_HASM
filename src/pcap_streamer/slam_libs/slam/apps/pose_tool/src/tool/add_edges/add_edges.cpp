/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "add_edges.hpp"

#include <nie/lidar/io/ba_graph/collection_helper.hpp>

#include "../common.hpp"

constexpr double kWeightGnssImuScale = 0.01;

void AddEdges(nie::io::PoseCollection* p_pose_collection) {
    auto& pose_collection = *p_pose_collection;
    pose_collection.header.Unset(nie::io::PoseHeader::kHasEdgeInformation);
    pose_collection.header.Set(nie::io::PoseHeader::kHasEdgeInformationPerRecord);

    if (pose_collection.poses.size() < 2) {
        return;
    }

    double const max_delta = GetMaximumDistanceThreshold(pose_collection);

    for (std::size_t i = 0; i < pose_collection.poses.size() - 1; ++i) {
        auto const& p0 = pose_collection.poses[i + 0];
        auto const& p1 = pose_collection.poses[i + 1];
        double delta = (p1.isometry.translation() - p0.isometry.translation()).norm();

        // Won't connect poses with too much distance between them
        if (delta > max_delta) {
            continue;
        }

        auto const ei = p0.isometry.TransformInverseLeft(p1.isometry);
        pose_collection.edges.emplace_back(nie::io::MakeEdgeRecord(p0.id, p1.id, ei, 1.0, kWeightGnssImuScale));
    }
}

void AddEdges() {
    nie::io::PoseCollection pose_collection;
    CHECK(ReadData(&pose_collection));
    AddEdges(&pose_collection);
    WriteData(pose_collection);
}
