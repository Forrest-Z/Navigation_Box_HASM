/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "nie/lidar/cloud.hpp"

#include <string>

namespace nie {

namespace io {

template <typename PointT>
Cloud<PointT> ReadLasHeader(std::string const& filename, std::chrono::weeks const& gps_week);

// The ReadLasPoints functions will reload the point cloud, removing the old one
template <typename PointT>
void ReadLasPoints(std::string const& filename, std::chrono::weeks const& gps_week, Cloud<PointT>* las_point_cloud);

template <typename PointT>
void ReadLasPoints(
    std::string const& filename,
    std::chrono::weeks const& gps_week,
    // returns true if kept, false if not
    std::function<bool(Eigen::Vector3d const& origin, PointT const& point, Timestamp_ns const& gps_time)> filter,
    Cloud<PointT>* las_point_cloud);

template <typename PointT>
Cloud<PointT> ReadLas(std::string const& filename, std::chrono::weeks const& gps_week);

}  // namespace io

}  // namespace nie

#include "las_reader.inl"
