/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <glog/logging.h>

namespace nie {

template <typename PointT>
Cloud<PointT> CloudFilter<PointT>::Filter(Cloud<PointT> const& cloud) {
    // Make sure everything is back to normal
    ResetParams();

    Cloud<PointT> result = cloud.CopyMetadataOnly();
    result.time_range() = cloud.time_range();

    // Filter the cloud
    grid_.setInputCloud(cloud.point_cloud_ptr());  // Input to be aligned with the target
    grid_.filter(result.point_cloud());            // Target cloud

    VLOG(3) << "CloudFilter reduced point cloud size from " << cloud.point_cloud().size() << " to "
            << result.point_cloud().size() << " points.";

    return result;
}

template <typename PointT>
void CloudFilter<PointT>::ResetParams() {
    grid_.setLeafSize(parameters_.voxel_size_x, parameters_.voxel_size_y, parameters_.voxel_size_z);
}

}  // namespace nie