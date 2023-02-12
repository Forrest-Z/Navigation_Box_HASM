/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/point_cloud.h>
#include <nie/core/time.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>

#include "nie/lidar/io/filenamer.hpp"

namespace nie {

class LidarLasWriter {
public:
    explicit LidarLasWriter(Filenamer const& las_filenamer) : las_filenamer_{las_filenamer} {}

    void ProcessSlice(
            pcl::PointCloud<pcl::PointXYZI> const& points,
            Eigen::Vector3d const& offset,
            std::vector<Timestamp_ns> const& timestamps);

    void ProcessSliceWithBounds(
            pcl::PointCloud<pcl::PointXYZI> const& points,
            Eigen::Vector3d const& offset,
            std::vector<Timestamp_ns> const& timestamps,
            PoseBbox const& bounds);

private:
    Filenamer const& las_filenamer_;
};

}  // namespace nie
