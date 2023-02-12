/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
/** Cloud Grid Splitter
 *
 * This class can be used to split an input cloud into a number of grids.
 * It does this by moving a box sized conditional filter over the total map.
 */
#pragma once

#include <vector>

#include <pcl/common/common.h>
#include <nie/lidar/cloud.hpp>

namespace nie {

template <typename PointT>
class CloudGridSplitter {
public:
    /**
     * Constructor
     * @param grid_size The size of the desired grid in meters.
     */
    explicit CloudGridSplitter(float grid_size) : grid_size_(grid_size) {}

    /**
     * This function implements the splitting of the input cloud into a vector of clouds.
     * @warning It only copies the bounds of original nie::Cloud, the time range is ignored in the output!
     * @param cloud The input cloud to be split
     * @result vector containing the split clouds
     * @remark returns empty vector if input cloud is empty
     * @remark the input cloud will be empty after splitting
     */
    std::vector<nie::Cloud<PointT>> Split(nie::Cloud<PointT>&& cloud);

private:
    float grid_size_;
};

}  // namespace nie

#include "cloud_grid_splitter.inl"