/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/core/time.hpp>

namespace nie::io::lidar {

/**
 * Definitions for a rotating lidar:
 *   Lidar Return  A detected return of a single laser pulse fired at moment τ, with azimuth α
 *   Lidar Packet  All lidar returns provided in a single network packet
 *   Lidar Sweep   All consecutive lidar returns taken with rotation angle γ <= α < γ + 360 (e.g. γ = 0)
 */
struct Returns {
    pcl::PointCloud<pcl::PointXYZI> points;
    std::vector<Timestamp_ns> timestamps;
};

struct Angles {
    std::vector<double> hor_angles;
    std::vector<double> ver_angles;
};

}  // namespace nie::io::lidar
