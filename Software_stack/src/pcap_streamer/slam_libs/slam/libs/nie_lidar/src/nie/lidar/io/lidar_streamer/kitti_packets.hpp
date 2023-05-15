/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/core/time.hpp>

namespace nie {

namespace io {

namespace kitti {

// Note that the end time of the current packet is the start time of the next packet.
struct Packet {
    nie::Timestamp_ns time_begin;
    nie::Timestamp_ns time_end;
    pcl::PointCloud<pcl::PointXYZI> points;
};

}  // namespace kitti

}  // namespace io

}  // namespace nie
