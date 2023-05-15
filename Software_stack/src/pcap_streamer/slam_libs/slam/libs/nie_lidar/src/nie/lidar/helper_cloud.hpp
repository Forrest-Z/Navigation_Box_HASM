/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/search/kdtree.h>
#include <nie/core/filesystem.hpp>

#include "cloud.hpp"
#include "nie/lidar/geometry/pose_bbox.hpp"

namespace nie {

template <typename PointT>
PoseBbox GetTotalExtent(std::vector<Cloud<PointT>> const& clouds, Eigen::Vector3d const& reference);

template <typename Iterator, typename Getter = std::function<PoseBbox const&(PoseBbox const&)>>
PoseBbox GetTotalExtent(
        Iterator begin,
        Iterator end,
        Eigen::Vector3d const& reference,
        Getter getter = [](PoseBbox const& b) -> PoseBbox const& { return b; });

template <typename PointT>
PoseBbox GetOverlappingExtent(std::vector<Cloud<PointT>> const& clouds, Eigen::Vector3d const& reference);

template <typename PointT>
PoseBbox CalculateOrientedBounds(pcl::PointCloud<PointT> const& cloud);

template <typename PointT>
PoseBbox CalculateOrientedBounds(nie::Cloud<PointT> const& cloud);

// The transformation supplied will be applied to the points in the cloud after which it is checked if the transformed
// point falls inside the bounding box. If not, then the points will be removed from the point cloud .
template <typename PointT>
void Filter(nie::Isometry3qd const&, Bboxf const&, nie::Cloud<PointT>*);

template <typename PointT>
typename pcl::search::KdTree<PointT>::Ptr GetKdTree(nie::Cloud<PointT> const& cloud, bool sorted = false);

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> GetPointClouds(std::vector<Cloud<PointT>> const& clouds);

template <typename Iterator, typename CloudGetter>
std::vector<PoseBbox> GetBounds(Iterator begin, Iterator end, CloudGetter cloud_getter);

template <typename PointT>
std::vector<PoseBbox> GetBounds(std::vector<Cloud<PointT>> const& clouds);

template <typename PointT>
void TransformCloud(nie::Isometry3qd const& T, nie::Cloud<PointT>* cloud);

template <typename PointT>
nie::Cloud<PointT> TransformCloud(nie::Cloud<PointT> const& cloud, nie::Isometry3qd const& T);

/// @brief Transforms the point cloud and change the origin.
/// @details Transforms the point cloud and change the origin. Transforming the cloud and changing the origin are
/// considered two independent things. The origin of the cloud is simply changed.
template <typename PointT>
nie::Cloud<PointT> TransformCloud(
        nie::Cloud<PointT> const& cloud, nie::Isometry3qd const& T, Eigen::Vector3d const& origin);

}  // namespace nie

#include "helper_cloud.inl"
