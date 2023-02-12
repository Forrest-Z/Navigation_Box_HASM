/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <Eigen/Core>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/time.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

namespace io {

namespace kitti {

std::vector<Timestamp_ns> ReadTimestamps(std::string const& path);

Isometry3qd ReadExtrinsics(std::string const& path);

using OxtsRecord = std::array<double, 30>;
OxtsRecord ReadOxts(std::string const& path);

// The 4 values are the x, y and z coordinates of the points in the lidar system. The fourth values is the reflectance
// and so on [0-1] interval
std::vector<Eigen::Vector4f> ReadPoints(std::string const& path);

nie::io::PoseCollection ReadGroundTruth(std::string const& path);

}  // namespace kitti

}  // namespace io

}  // namespace nie
