/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_point_interpolator.hpp"

#include <algorithm>  // for std::upper_bound

#include <nie/core/geometry/interpolation.hpp>

namespace nie {

namespace {

inline double ComputeRatio(
        Timestamp_ns const& pose_timestamp_before,
        Timestamp_ns const& pose_timestamp_after,
        Timestamp_ns const& lidar_timestamp) {
    //
    return static_cast<double>((lidar_timestamp - pose_timestamp_before).count()) /
           static_cast<double>((pose_timestamp_after - pose_timestamp_before).count());
}

}  // namespace

LidarPointInterpolator::LidarPointInterpolator(Iterator begin, Iterator end)
    : range_finder_{begin, end, PoseTimestampGetter{}},
      calibration_isometry_before_{begin->isometry},
      calibration_isometry_after_{end->isometry} {}

void LidarPointInterpolator::TransformPoints(
        std::vector<io::PoseRecord> const& lidar_poses,
        Eigen::Vector3d const& offset,
        pcl::PointCloud<pcl::PointXYZI>* points,
        std::vector<Timestamp_ns>* timestamps) {
    // First transform all lidar points to the world
    LidarPointInterpolator interpolator(lidar_poses.cbegin(), lidar_poses.cend());

    std::vector<std::size_t> remove_indices;
    for (std::size_t i = 0; i < points->size(); ++i) {
        auto [isometry, success] = interpolator((*timestamps)[i]);
        if (success) {
            (*points)[i].getVector3fMap() =
                    (isometry * (*points)[i].getVector3fMap().cast<double>() - offset).cast<float>();
        } else {
            remove_indices.push_back(i);
        }
    }
    points->erase(RemoveIf(remove_indices.cbegin(), remove_indices.cend(), &(points->points)), points->end());
    points->width = static_cast<std::uint32_t>(points->size());
    timestamps->erase(RemoveIf(remove_indices.cbegin(), remove_indices.cend(), timestamps), timestamps->end());
    VLOG(1) << "Skipped " << remove_indices.size() << " lidar points, leaving " << points->size() << " remaining.";
}

std::pair<Isometry3qd, bool> LidarPointInterpolator::operator()(Timestamp_ns const& lidar_timestamp) {
    auto it_before_prev = range_finder_.it_before();
    auto it_after_prev = range_finder_.it_after();

    DVLOG(9) << "Interpolating lidar_timestamp = " << lidar_timestamp << " in range " << it_before_prev->timestamp
             << " to " << it_after_prev->timestamp;

    auto [it_before, it_after, success] = range_finder_(lidar_timestamp);
    if (!success) {
        VLOG(4) << "Lidar timestamp " << lidar_timestamp << ": out of range.";
        return {Isometry3qd{}, false};
    }

    if (it_before_prev != it_before) {
        // left bound has changed so we have to compute a new lidar-to-world isometry.
        calibration_isometry_before_ = it_before->isometry;
    }
    if (it_after_prev != it_after) {
        // right bound has changed so we have to compute a new lidar-to-world isometry.
        calibration_isometry_after_ = it_after->isometry;
    }

    if (it_before == it_after) {
        return {calibration_isometry_before_, true};
    }

    double const ratio = ComputeRatio(it_before->timestamp, it_after->timestamp, lidar_timestamp);
    return {Interpolate(calibration_isometry_before_, calibration_isometry_after_, ratio), true};
}

}  // namespace nie
