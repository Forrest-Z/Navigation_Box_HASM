/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "cloud.hpp"

#include <pcl/filters/voxel_grid.h>

namespace nie {

template <typename PointT>
class CloudFilter {
public:
    struct Parameters {
        // Non-uniform voxel size
        Parameters(double voxel_size_x, double voxel_size_y, double voxel_size_z)
            : voxel_size_x(voxel_size_x), voxel_size_y(voxel_size_y), voxel_size_z(voxel_size_z) {}

        // Uniform voxel size
        explicit Parameters(double voxel_size) : Parameters(voxel_size, voxel_size, voxel_size) {}

        // Default voxel size
        Parameters() : Parameters(0.2) {}

        double voxel_size_x;
        double voxel_size_y;
        double voxel_size_z;
    };

    explicit CloudFilter(Parameters parameters = Parameters()) : parameters_(parameters), grid_() { ResetParams(); }

    // TODO(jbr) Bring back with pointer interface
    Cloud<PointT> Filter(Cloud<PointT> const& cloud);

private:
    void ResetParams();

    Parameters parameters_;

    pcl::VoxelGrid<PointT> grid_;
};

}  // namespace nie

#include "cloud_filter.inl"
