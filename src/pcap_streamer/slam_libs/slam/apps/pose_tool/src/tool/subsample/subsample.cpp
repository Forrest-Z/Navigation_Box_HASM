/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "subsample.hpp"

#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/formats/csv/csv_record.hpp>
#include <nie/formats/csv/csv_record_io.hpp>

#include "../common.hpp"

DEFINE_double(vertex_distance_threshold, 0.0, "Minimum distance between vertices.");
DEFINE_validator(vertex_distance_threshold, nie::ValidateLargerOrEqualToZero);

void Subsample() {
    nie::io::PoseCollection pose_collection;
    CHECK(ReadData(&pose_collection));

    auto out_file = GetAndCheckOutPathsForExtensionOrFatal(".csv");
    std::vector<nie::io::PoseId> ids;

    double distance = 0.0;
    for (std::size_t i = 0; i < pose_collection.poses.size(); ++i) {
        auto const& pose_c = pose_collection.poses[i];

        if (i > 0) {
            auto const& pose_p = pose_collection.poses[i - 1];
            distance += (pose_c.isometry.translation() - pose_p.isometry.translation()).norm();

            if (distance > FLAGS_vertex_distance_threshold) {
                distance = 0.0;
            }
        }

        if (distance == 0.0) {
            ids.push_back(pose_c.id);
        }
    }

    LOG(INFO) << "Subsampled " << ids.size() << " vertices from " << pose_collection.poses.size() << " vertices.";

    nie::CsvRecord<nie::io::PoseId> csv_records{ids, ',', "id"};
    nie::WriteCsvRecord(out_file.string(), csv_records);
}
