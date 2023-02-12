/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <fstream>
#include <regex>

#include <glog/logging.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <pdal/PointView.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>

#include <nie/core/filesystem.hpp>
#include <nie/core/glog.hpp>
#include <nie/core/string.hpp>

#include "nie/lidar/geometry/pose_bbox.hpp"

namespace nie {

namespace io {

namespace detail {

template <typename PointT>
void SetXyz(
        pdal::LasHeader const& header,
        pdal::PointViewPtr const& point_view_ptr,
        size_t const index,
        PointT* point_pcl) {
    point_pcl->x =
            static_cast<float>(point_view_ptr->getFieldAs<double>(pdal::Dimension::Id::X, index) - header.offsetX());
    point_pcl->y =
            static_cast<float>(point_view_ptr->getFieldAs<double>(pdal::Dimension::Id::Y, index) - header.offsetY());
    point_pcl->z =
            static_cast<float>(point_view_ptr->getFieldAs<double>(pdal::Dimension::Id::Z, index) - header.offsetZ());
}

template <typename PointT>
inline PointT PointLasToPcl(pdal::LasHeader const& header, pdal::PointViewPtr const& point_view_ptr, size_t index);

template <>
inline pcl::PointXYZ PointLasToPcl<pcl::PointXYZ>(
        pdal::LasHeader const& header, pdal::PointViewPtr const& point_view_ptr, size_t index) {
    pcl::PointXYZ point_pcl;
    SetXyz(header, point_view_ptr, index, &point_pcl);
    return point_pcl;
}

template <>
inline pcl::PointXYZI PointLasToPcl<pcl::PointXYZI>(
        pdal::LasHeader const& header, pdal::PointViewPtr const& point_view_ptr, size_t index) {
    pcl::PointXYZI point_pcl;
    SetXyz(header, point_view_ptr, index, &point_pcl);
    // Int to float, do we need to do more?
    point_pcl.intensity = point_view_ptr->getFieldAs<float>(pdal::Dimension::Id::Intensity, index);
    return point_pcl;
}

template <typename PointT>
struct NoOpFilter {
    inline constexpr bool operator()(Eigen::Vector3d const&, PointT const&, Timestamp_ns const&) const { return false; }
};

/// The gps week is required as the las file only contains the timestamp per point as gps time in week.
/// Therefore the gps week is required to calculate the complete timestamp for every point.
template <typename PointT, typename Filter = NoOpFilter<PointT>>
class LasReaderWrapper {
public:
    LasReaderWrapper(std::string const& path, std::chrono::weeks const gps_week) try : gps_week_(gps_week),
                                                                                       options_(),
                                                                                       table_(),
                                                                                       reader_(),
                                                                                       header_(),
                                                                                       point_view_set_(),
                                                                                       point_view_(),
                                                                                       origin_(),
                                                                                       filter_() {
        // PDAL initialization
        options_.add("filename", path);
        reader_.setOptions(options_);
        reader_.prepare(table_);
        point_view_set_ = reader_.execute(table_);
        point_view_ = *point_view_set_.begin();
        header_ = reader_.header();
        origin_ = {header_.offsetX(), header_.offsetY(), header_.offsetZ()};
    } catch (std::exception const& e) {
        nie::LogExceptionAndFatal(e);
    }

    LasReaderWrapper(std::string const& path, std::chrono::weeks const gps_week, Filter filter) try
        : gps_week_(gps_week),
          options_(),
          table_(),
          reader_(),
          header_(),
          point_view_set_(),
          point_view_(),
          origin_(),
          filter_(std::move(filter)) {
        // PDAL initialization
        options_.add("filename", path);
        reader_.setOptions(options_);
        reader_.prepare(table_);
        point_view_set_ = reader_.execute(table_);
        point_view_ = *point_view_set_.begin();
        header_ = reader_.header();
        origin_ = {header_.offsetX(), header_.offsetY(), header_.offsetZ()};
    } catch (std::exception const& e) {
        nie::LogExceptionAndFatal(e);
    }

    // Input is gps time in week in seconds
    Timestamp_ns CreateTimestamp(double const& gps_time_in_week) const {
        // Convert time from seconds to nanoseconds
        auto const gps_time_in_week_ns = RepresentDoubleAsDuration<std::chrono::nanoseconds>(gps_time_in_week);
        // Create GPS week time object
        auto const gps_week_time = GPSWeekTime<std::chrono::nanoseconds>{gps_week_, gps_time_in_week_ns};
        // Convert to gps timestamp
        return nie::ToGPSTime(gps_week_time);
    }

    nie::Bboxf bbox() const {
        return {{static_cast<float>(header_.minX() - header_.offsetX()),
                 static_cast<float>(header_.minY() - header_.offsetY()),
                 static_cast<float>(header_.minZ() - header_.offsetZ())},
                {static_cast<float>(header_.maxX() - header_.offsetX()),
                 static_cast<float>(header_.maxY() - header_.offsetY()),
                 static_cast<float>(header_.maxZ() - header_.offsetZ())}};
    }

    Eigen::Vector3d const& origin() const { return origin_; }

    PoseBbox bounds() const { return {origin(), bbox()}; };

    void ReadPointCloud(
            typename pcl::PointCloud<PointT>::Ptr* pp_point_cloud, std::vector<Timestamp_ns>* p_timestamps) {
        *pp_point_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>& point_cloud = **pp_point_cloud;
        std::vector<Timestamp_ns>& timestamps = *p_timestamps;
        auto& points = point_cloud.points;
        points.reserve(header_.pointCount());

        for (size_t i = 0; i < point_view_->size(); ++i) {
            PointT point = PointLasToPcl<PointT>(header_, point_view_, i);
            Timestamp_ns const timestamp =
                    CreateTimestamp(point_view_->getFieldAs<double>(pdal::Dimension::Id::GpsTime, i));
            if (!filter_(origin(), point, timestamp)) {
                points.emplace_back(point);
                timestamps.push_back(timestamp);
            }
        }

        // CAUTION: Setting dimensions should be after filling the points !!!
        point_cloud.width = static_cast<std::uint32_t>(points.size());
        point_cloud.height = 1;
    }

private:
    std::chrono::weeks const gps_week_;
    pdal::Options options_;
    pdal::PointTable table_;
    pdal::LasReader reader_;
    pdal::LasHeader header_;
    pdal::PointViewSet point_view_set_;
    pdal::PointViewPtr point_view_;
    Eigen::Vector3d origin_;
    Filter filter_;
};

}  // namespace detail

}  // namespace io

}  // namespace nie
