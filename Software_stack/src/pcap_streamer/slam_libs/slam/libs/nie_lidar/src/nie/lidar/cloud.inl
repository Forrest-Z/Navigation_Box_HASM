/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <pcl/common/centroid.h>

namespace nie {

template <typename PointT>
Cloud<PointT> Cloud<PointT>::CopyWithOrigin(Eigen::Vector3d const& origin) const {
    Cloud<PointT> updated = Copy();
    updated.ChangeOrigin(origin);
    return updated;
}

template <typename PointT>
void Cloud<PointT>::ChangeOrigin(Eigen::Vector3d const& target_origin) {
    Eigen::Vector3f delta_f = (origin().translation() - target_origin).template cast<float>();
    bounds_ = bounds().CopyWithOrigin(target_origin);

    for (auto& p : point_cloud().points) {
        p.template getVector3fMap() += delta_f;
    }
}

namespace detail {

template <typename UnaryPredicate>
std::size_t FindIfIndex(std::size_t index_begin, std::size_t index_end, UnaryPredicate p) {
    for (std::size_t i = index_begin; i != index_end; ++i) {
        if (p(i)) return i;
    }
    return index_end;
}

}  // namespace detail

/// @brief Filter by point. [](PointT const&)->bool
template <typename PointT>
template <typename UnaryPredicate>
void Cloud<PointT>::FilterPoints(UnaryPredicate const& filter) {
    auto& points = point_cloud_->points;
    auto filter_with_index = [&points, &filter](std::size_t const& index) -> bool { return filter(points[index]); };
    FilterIndices(filter_with_index);
}

/// @brief Filter by index. [](std::size_t  const&)->bool
// This version has a custom index based remove_if implementation so it can be re-used by FilterPoints.
// Should we add sibling vectors again, we can more efficiently move those too.
template <typename PointT>
template <typename UnaryPredicate>
void Cloud<PointT>::FilterIndices(UnaryPredicate const& filter) {
    auto& points = point_cloud_->points;

    // Custom std::remove_if but then with indices.
    std::size_t index_begin = detail::FindIfIndex(0, points.size(), filter);

    if (index_begin == points.size()) return;

    for (std::size_t i = index_begin; i != points.size(); ++i) {
        if (!filter(i)) {
            points[index_begin] = std::move(points[i]);
            ++index_begin;
        }
    }
    // end remove_if

    points.erase(points.begin() + index_begin, points.end());

    point_cloud_->width = static_cast<std::uint32_t>(point_cloud_->size());
    point_cloud_->height = 1;
    bounds_.UpdateBbox(*point_cloud_);
}

}  // namespace nie
