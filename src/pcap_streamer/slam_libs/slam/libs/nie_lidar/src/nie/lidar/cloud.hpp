/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <iostream>
#include <numeric>
#include <vector>

#include <glog/logging.h>
#include <pcl/common/io.h>    // pcl::copyPointCloud
#include <pcl/point_types.h>  // pcl::PointXYZ
#include <Eigen/Eigen>
#include <nie/core/time.hpp>

#include "nie/lidar/geometry/pose_bbox.hpp"

namespace nie {

/// @brief The cloud class contains a pcl::PointCloud<PointT> and corresponding metadata.
/// @details The copy constructor function as shallow copies while the Copy<...> functions create a deep copy.
template <typename PointT>
class Cloud {
public:
    using Point = PointT;
    using PointCloud = pcl::PointCloud<Point>;
    using PointCloudPtr = typename pcl::PointCloud<Point>::Ptr;

    Cloud() : point_cloud_(new PointCloud()) {}

    // TODO(jbr) This is the header only version. Perhaps make a header class for improved intention.
    Cloud(PoseBbox bounds, std::pair<nie::Timestamp_ns, nie::Timestamp_ns> time_range = {})
        : bounds_(std::move(bounds)), time_range_(std::move(time_range)), point_cloud_(new PointCloud()) {}

    Cloud(PoseBbox bounds, std::pair<nie::Timestamp_ns, nie::Timestamp_ns> time_range, PointCloudPtr point_cloud)
        : bounds_(std::move(bounds)), time_range_(std::move(time_range)), point_cloud_(std::move(point_cloud)) {}

    Isometry3qd const& origin() const { return bounds_.origin(); }
    Isometry3qd& origin() { return bounds_.origin(); }
    PoseBbox const& bounds() const { return bounds_; }
    PoseBbox& bounds() { return bounds_; }
    std::pair<Timestamp_ns, Timestamp_ns> const& time_range() const { return time_range_; }
    std::pair<Timestamp_ns, Timestamp_ns>& time_range() { return time_range_; }

    PointCloud const& point_cloud() const { return *point_cloud_; }
    PointCloud& point_cloud() { return *point_cloud_; }
    PointCloudPtr const& point_cloud_ptr() const { return point_cloud_; }
    PointCloudPtr& point_cloud_ptr() { return point_cloud_; }

    PoseBbox Intersection(Cloud const& other) const {
        auto bounds_a = bounds();
        auto bounds_b = other.bounds().CopyWithOrigin(origin().translation());
        return PoseBbox(origin().translation(), (bounds_a.bbox() & bounds_b.bbox()));
    }

    bool Intersects(Cloud const& other) const {
        Bboxf box_union = Intersection(other).bbox();
        // If all components in the range are positive, then the clouds overlap
        return (box_union.Range().array() > 0.).all();
    }

    // Anything with a non 1 to 1 correspondence to a point
    template <typename PointK = PointT>
    Cloud<PointK> CopyMetadataOnly() const {
        return Cloud<PointK>{bounds_, time_range_};
    }
    Cloud<PointT> Copy() const {
        Cloud<PointT> copy = CopyMetadataOnly();
        // This is the only reason we have a custom copy (otherwise it only copies the pointer)
        pcl::copyPointCloud(point_cloud(), copy.point_cloud());
        return copy;
    }
    Cloud<PointT> CopyWithOrigin(Eigen::Vector3d const& origin) const;

    void ChangeOrigin(Eigen::Vector3d const& origin);

    template <typename UnaryPredicate>
    void FilterPoints(UnaryPredicate const& filter);
    template <typename UnaryPredicate>
    void FilterIndices(UnaryPredicate const& filter);

private:
    /// @brief reference frame and bounding box of the point cloud
    PoseBbox bounds_;
    /// @brief timestamp range of the points from the original las file
    std::pair<nie::Timestamp_ns, nie::Timestamp_ns> time_range_;
    /// @brief 3d points as floats [meter] excl. the data is offset by bounds_
    PointCloudPtr point_cloud_;
};

}  // namespace nie

#include "cloud.inl"
