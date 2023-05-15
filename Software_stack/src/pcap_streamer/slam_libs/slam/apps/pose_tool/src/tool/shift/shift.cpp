/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "shift.hpp"

#include <nie/core/gflags.hpp>
#include <nie/core/string.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "tool/io.hpp"

DEFINE_double(shift_x, 0.0, "Distance in meters to shift poses in x direction.");
DEFINE_double(shift_y, 0.0, "Distance in meters to shift poses in y direction.");
DEFINE_double(shift_z, 0.0, "Distance in meters to shift poses in z direction.");
DEFINE_int64(shift_time, 0, "Duration in microseconds to shift poses.");

void Shift() {
    nie::io::PoseCollection pose_collection;
    // Read the data into the pose_collection
    CHECK(ReadData(&pose_collection));

    // Check if input says to shift position and report
    bool shift_position = false;
    if (FLAGS_shift_x || FLAGS_shift_y || FLAGS_shift_z) {
        shift_position = true;
        LOG(INFO) << "Shifting positions";
    }
    // Check if we are shifting time and report
    if (FLAGS_shift_time) {
        LOG(INFO) << "Shifting " << FLAGS_shift_time << " microseconds";
    }
    // Only shift if we have to
    if (shift_position || FLAGS_shift_time) {
        // Store the translation
        Eigen::Vector3d const t_pos{FLAGS_shift_x, FLAGS_shift_y, FLAGS_shift_z};
        auto const dt = std::chrono::microseconds(FLAGS_shift_time);
        // Loop over poses
        for (auto& pose : pose_collection.poses) {
            if (shift_position) {
                pose.isometry.translation() += t_pos;
            }
            if (FLAGS_shift_time) {
                pose.timestamp += dt;
            }
        }
        // Otherwise report
    } else {
        LOG(INFO) << "PoseCollection was not shifted.";
    }

    // Output data to file dest
    WriteData(pose_collection);
}
