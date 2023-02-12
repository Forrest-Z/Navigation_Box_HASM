/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <nie/core/string.hpp>

#include "helper_las.hpp"

namespace nie {
namespace io {
namespace detail {

template <typename PointT, typename Filter>
void ReadLasPoints(LasReaderWrapper<PointT, Filter>* w, Cloud<PointT>* las_point_cloud) {
    typename pcl::PointCloud<PointT>::Ptr point_cloud;
    std::vector<Timestamp_ns> timestamps;
    w->ReadPointCloud(&point_cloud, &timestamps);
    CHECK(!point_cloud.get()->points.empty()) << "No points read.";
    auto min_max = std::minmax_element(timestamps.begin(), timestamps.end());
    *las_point_cloud =
            Cloud<PointT>(w->bounds(), std::make_pair(*min_max.first, *min_max.second), std::move(point_cloud));
}

}  // namespace detail

template <typename PointT>
Cloud<PointT> ReadLasHeader(std::string const& filename, std::chrono::weeks const& gps_week) {
    detail::LasReaderWrapper<PointT> w(filename, gps_week);
    return Cloud<PointT>(w.bounds());
}

template <typename PointT>
void ReadLasPoints(std::string const& filename, std::chrono::weeks const& gps_week, Cloud<PointT>* las_point_cloud) {
    detail::LasReaderWrapper<PointT> w(filename, gps_week);
    detail::ReadLasPoints(&w, las_point_cloud);
}

template <typename PointT, typename Filter>
void ReadLasPoints(
        std::string const& filename,
        std::chrono::weeks const& gps_week,
        Filter filter,
        Cloud<PointT>* las_point_cloud) {
    detail::LasReaderWrapper<PointT, Filter> w(filename, gps_week, filter);
    detail::ReadLasPoints(&w, las_point_cloud);
}

template <typename PointT>
Cloud<PointT> ReadLas(std::string const& filename, std::chrono::weeks const& gps_week) {
    detail::LasReaderWrapper<PointT> w(filename, gps_week);
    typename pcl::PointCloud<PointT>::Ptr point_cloud;
    std::vector<Timestamp_ns> timestamps;
    w.ReadPointCloud(&point_cloud, &timestamps);
    auto min_max = std::minmax_element(timestamps.begin(), timestamps.end());
    return Cloud<PointT>(w.bounds(), std::make_pair(*min_max.first, *min_max.second), std::move(point_cloud));
}

}  // namespace io

}  // namespace nie
