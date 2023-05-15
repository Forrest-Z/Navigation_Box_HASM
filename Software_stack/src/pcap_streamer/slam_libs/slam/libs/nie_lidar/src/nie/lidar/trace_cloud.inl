/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "euclidean_distance.hpp"
#include "slice.hpp"

namespace nie {

template <typename PointT>
nie::Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<nie::io::PoseRecord> const& trace, nie::Cloud<PointT> const& cloud) {
    return MakeTraceCloudFromTrace(trace, cloud.origin(), cloud.time_range().first, cloud.time_range().second);
}

template <typename PointT>
double TraceLength(std::vector<nie::io::PoseRecord> const& full_trace, nie::Cloud<PointT> const& cloud) {
    auto lb = std::lower_bound(full_trace.cbegin(), full_trace.cend(), cloud.time_range().first);
    auto ub = std::upper_bound(lb, full_trace.cend(), cloud.time_range().second);

    return SliceByDistance(lb, ub, std::numeric_limits<double>::max(), EuclideanDistance{}).second;
}

}  // namespace nie
