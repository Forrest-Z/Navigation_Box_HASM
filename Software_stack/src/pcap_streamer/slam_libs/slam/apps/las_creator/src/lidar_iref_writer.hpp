/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/io/filenamer.hpp>

namespace nie {

class LidarIrefWriter {
public:
    LidarIrefWriter(Filenamer const& las_filenamer, std::vector<io::PoseRecord> const& poses);

    void ProcessSlice(std::vector<Timestamp_ns> const& timestamps);

private:
    Filenamer const& las_filenamer_;
    std::vector<io::PoseRecord> const& poses_;
    io::InfoRefCollectionStreamWriter iref_collection_writer_;
};

}  // namespace nie
