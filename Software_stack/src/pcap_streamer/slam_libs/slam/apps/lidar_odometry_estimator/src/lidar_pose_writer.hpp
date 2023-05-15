/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph.hpp>
#include <nie/lidar/io/filenamer.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

namespace nie {

class LidarPoseWriter {
public:
    LidarPoseWriter(std::string const& las_filenamer, double loam_sd_scale);

    void ProcessSweep(io::PoseRecord const& pose);

private:
    io::PoseCollectionStreamWriter pose_collection_writer_;

    bool has_previous_;
    double const loam_sd_scale_;
    io::PoseRecord previous_pose_;
};

}  // namespace nie
