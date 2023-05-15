/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/search/kdtree.h>
#include <nie/lidar/cloud.hpp>
#include <nie/lidar/feature/surface.hpp>

namespace detail {
inline bool RejectMatrix(Eigen::Matrix3d const& cov) {
    // Compute the SVD (covariance matrix is symmetric so U = V')
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov);
    return (svd.singularValues()(2) / svd.singularValues().sum()) > 0.125;
}

template <typename PointT>
typename pcl::search::KdTree<PointT>::Ptr GetKdTree(nie::Cloud<PointT> const& cloud) {
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud.point_cloud_ptr());
    return tree;
}
}  // namespace detail

template <typename PointT>
void FilterWithCovariance(nie::Cloud<PointT>* cloud) {
    auto tree = detail::GetKdTree(*cloud);
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> covariances;
    CalculatePointCovariances(*tree, &covariances);
    auto filter = [&covariances](std::size_t const& i) -> bool { return detail::RejectMatrix(covariances[i]); };
    size_t size_original = cloud->point_cloud().size();
    cloud->FilterIndices(filter);
    VLOG(3) << "FilterWithCovariance reduced point cloud size from " << size_original << " to "
            << cloud->point_cloud().size() << ".";
}
