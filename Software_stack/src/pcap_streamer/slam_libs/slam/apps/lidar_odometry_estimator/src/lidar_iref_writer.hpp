/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <boost/filesystem.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/lidar/io/filenamer.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

namespace nie {

class LidarIrefWriter {
public:
    explicit LidarIrefWriter(Filenamer const& las_filenamer);

    void ProcessSweep(io::PoseRecord const& pose);

private:
    Filenamer const& las_filenamer_;
    io::InfoRefCollectionStreamWriter iref_collection_writer_;
};

}  // namespace nie
