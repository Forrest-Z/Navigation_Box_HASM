/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/cloud.hpp>

namespace nie {

// A cloud object is created using the trace range supplied. As expected the timestamp range is set according.
nie::Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<nie::io::PoseRecord>::const_iterator const trace_begin,
        std::vector<nie::io::PoseRecord>::const_iterator const trace_end,
        nie::Isometry3qd const& origin = nie::Isometry3qd::Identity());

// A cloud object is created from the poses supplied that lay within the supplied time range. The timestamp range is set
// to the timestamps of the poses that are actually in the interval.
nie::Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<nie::io::PoseRecord> const& trace,
        nie::Isometry3qd const& origin,
        nie::Timestamp_ns const& start_time,
        nie::Timestamp_ns const& end_time);

template <typename PointT>
nie::Cloud<pcl::PointXYZ> MakeTraceCloudFromTrace(
        std::vector<nie::io::PoseRecord> const& trace, nie::Cloud<PointT> const& cloud);

template <typename PointT>
double TraceLength(std::vector<nie::io::PoseRecord> const& trace, nie::Cloud<PointT> const& cloud);

}  // namespace nie

#include "trace_cloud.inl"
