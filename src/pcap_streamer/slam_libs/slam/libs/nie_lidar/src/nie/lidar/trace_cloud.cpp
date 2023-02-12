/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "trace_cloud.hpp"

namespace nie {

namespace detail {

void MakeCloudFromTrace(
        std::vector<nie::io::PoseRecord>::const_iterator const trace_begin,
        std::vector<nie::io::PoseRecord>::const_iterator const trace_end,
        nie::Isometry3qd const& origin,
        pcl::PointCloud<pcl::PointXYZ>* cloud) {
    cloud->points.reserve(std::distance(trace_begin, trace_end));

    bool const other_origin = origin == nie::Isometry3qd::Identity();
    Isometry3md const origin_inv{origin.Inversed()};
    for (auto it = trace_begin; it < trace_end; ++it) {
        cloud->points.emplace_back(
                MakePointXYZ(other_origin ? origin_inv * it->isometry.translation() : it->isometry.translation()));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

}  // namespace detail

Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<nie::io::PoseRecord>::const_iterator const trace_begin,
        std::vector<nie::io::PoseRecord>::const_iterator const trace_end,
        nie::Isometry3qd const& origin) {
    // Create Cloud object
    Cloud<pcl::PointXYZ> cloud{};
    cloud.origin() = origin;
    detail::MakeCloudFromTrace(trace_begin, trace_end, origin, &cloud.point_cloud());
    cloud.bounds().bbox() = Bboxf::Create(cloud.point_cloud());
    cloud.time_range() = std::make_pair(trace_begin->timestamp, (trace_end - 1)->timestamp);

    return cloud;
}

Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<io::PoseRecord> const& trace,
        Isometry3qd const& origin,
        Timestamp_ns const& start_time,
        Timestamp_ns const& end_time) {
    CHECK(!trace.empty()) << "Unexpected empty trace.";
    // Returns the first element after start_time (when not present)
    auto lb = std::lower_bound(trace.begin(), trace.end(), start_time);
    CHECK(lb != trace.end()) << "The start_time not available within trace.";

    // Returns the first element after end_time (when not present)
    auto ub = std::lower_bound(lb, trace.end(), end_time);
    CHECK(ub != trace.end()) << "The end_time not available within trace.";

    // Copy all points
    auto n_points = static_cast<std::uint32_t>(std::distance(lb, ub));
    CHECK(n_points != 0) << "Filtered trace is empty.";

    return MakeTraceCloudFromTrace(lb, ub, origin);
}

}  // namespace nie
