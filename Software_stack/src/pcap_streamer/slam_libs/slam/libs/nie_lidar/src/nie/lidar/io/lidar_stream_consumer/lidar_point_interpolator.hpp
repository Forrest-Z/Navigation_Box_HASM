/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "nie/lidar/range_finder.hpp"

namespace nie {

class LidarPointInterpolator {
public:
    using Iterator = std::vector<io::PoseRecord>::const_iterator;

    explicit LidarPointInterpolator(Iterator begin, Iterator end);

    static void TransformPoints(
            std::vector<io::PoseRecord> const& lidar_poses,
            Eigen::Vector3d const& offset,
            pcl::PointCloud<pcl::PointXYZI>* points,
            std::vector<Timestamp_ns>* timestamps);

    std::pair<Isometry3qd, bool> operator()(Timestamp_ns const& lidar_timestamp);

private:
    // TODO: The implementation in this class uses the range finder. It is likely that can be replaced with the
    // InterpolateIsometry functions in nie/formats/ba_graph/pose_collection.hpp.

    // Adapter for converting io::PoseRecord to timestamp
    struct PoseTimestampGetter {
        Timestamp_ns const& operator()(io::PoseRecord const& pose) { return pose.timestamp; }
    };
    RangeFinder<io::PoseRecord, std::less, PoseTimestampGetter> range_finder_;

    // These serve as a "cache" so that we do not recompute the isometry every time we interpolate.
    Isometry3qd calibration_isometry_before_;
    Isometry3qd calibration_isometry_after_;
};

}  // namespace nie
