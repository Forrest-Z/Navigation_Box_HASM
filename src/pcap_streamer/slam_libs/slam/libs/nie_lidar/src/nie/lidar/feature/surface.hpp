/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LIDAR_FEATURE_SURFACE_HPP
#define NIE_LIDAR_FEATURE_SURFACE_HPP

#include <pcl/search/kdtree.h>
#include <Eigen/Eigen>

template <typename PointT>
void CalculatePointCovariances(
    pcl::search::KdTree<PointT> const& kdtree,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>* p_covariances);

#include "surface.inl"

#endif  // NIE_LIDAR_FEATURE_SURFACE_HPP
