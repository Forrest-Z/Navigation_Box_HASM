/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cmath>

#include <glog/logging.h>
#include <pcl/common/common.h>

#include <nie/core/spatial_grid.hpp>

namespace nie {

template <typename PointT>
std::vector<nie::Cloud<PointT>> CloudGridSplitter<PointT>::Split(nie::Cloud<PointT>&& cloud) {
    // Check input cloud
    if (cloud.point_cloud().empty()) {
        LOG(WARNING) << "Input cloud is empty. Returning empty vector.";
        return {};
    }

    // Obtain grid cells using a spatial grid
    std::vector<std::vector<std::size_t>> const binned_data =
            nie::CreateIndicesSpatialGrid<2, float>(grid_size_, cloud.point_cloud().points, [&cloud](PointT const& p) {
                return p.getVector3fMap().template head<2>();
            }).GetBinnedData();

    // Create output cloud vector grid
    std::vector<nie::Cloud<PointT>> result(binned_data.size());

    // Fill output clouds
    for (size_t cell_index = 0; cell_index < binned_data.size(); ++cell_index) {
        for (auto const point_index : binned_data[cell_index]) {
            result[cell_index].point_cloud().points.push_back(std::move(cloud.point_cloud().points[point_index]));
        }
    }

    // Remove empty clouds
    result.erase(
            std::remove_if(
                    result.begin(),
                    result.end(),
                    [](nie::Cloud<PointT> const& c) { return c.point_cloud().points.empty(); }),
            result.end());

    // Fill the clouds with relevant information
    std::for_each(result.begin(), result.end(), [cloud](nie::Cloud<PointT>& c) {
        c.point_cloud().width = c.point_cloud().points.size();
        c.point_cloud().height = 1;
        c.point_cloud().is_dense = true;
        c.bounds().origin() = cloud.bounds().origin();
        c.bounds().UpdateBbox(c.point_cloud());
        Eigen::Vector3d t = cloud.origin().translation();
        c.point_cloud().sensor_origin_ << t.cast<float>(), 1;
    });

    return result;
}

}  // namespace nie