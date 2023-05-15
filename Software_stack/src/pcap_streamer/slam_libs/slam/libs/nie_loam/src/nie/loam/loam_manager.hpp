/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LOAM_LOAM_MANAGER_HPP
#define NIE_LOAM_LOAM_MANAGER_HPP

#include <string>
#include <vector>

#include <nie/core/time.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

#include "nie/loam/mapping/laser_mapping.hpp"
#include "nie/loam/odometry/laser_odometry.hpp"
#include "nie/loam/registration/multi_scan_registration.hpp"
#include "nie/loam/transform_maintenance/transform_maintenance.hpp"

namespace nie {

namespace detail {

inline loam::PointCloudMessage LidarReturnsToPointCloudMessage(nie::io::lidar::Returns const& returns, uint32_t seq) {
    return loam::PointCloudMessage{
            loam::MessageHeader{
                    seq,
                    std::chrono::clock_cast<nie::Timestamp_ns::clock>(returns.timestamps.front()),
                    "returns_frame"},
            returns.points};
}

template <class PointT>
inline loam::PointCloudMessage CreatePointCloudMessage(
        pcl::PointCloud<PointT> point_cloud, nie::Timestamp_ns stamp, uint32_t seq, std::string const& frame_id) {
    pcl::PointCloud<pcl::PointXYZI> point_cloud_ret;
    // Message clouds are for now always XYZI
    pcl::copyPointCloud(point_cloud, point_cloud_ret);
    return loam::PointCloudMessage{loam::MessageHeader{seq, stamp, frame_id}, point_cloud_ret};
}

}  // namespace detail

class LoamManager {
public:
    explicit LoamManager(
            loam::MultiScanMapper const& multi_scan_mapper,
            loam::RegistrationParams const& config = loam::RegistrationParams(),
            std::string const& imr_file_path = "");

    void ProcessLidarReturns(nie::io::lidar::Returns const& lidar_returns);
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI> const& MappingLaserCloud() const { return mapping_laser_cloud_; }
    [[nodiscard]] loam::Odometry const& MappedOdometry() const { return mapping_odometry_; }

private:
    uint32_t seq_ = 0;

    // Loam components
    loam::MultiScanRegistration multi_scan_registration_;
    loam::LaserOdometry laser_odometry_;
    loam::LaserMapping laser_mapping_;
    loam::TransformMaintenance transform_maintenance_;

    // Resulting accumulated cloud
    pcl::PointCloud<pcl::PointXYZI> mapping_laser_cloud_;
    // Odometry measurement obtained from the Laser Mapping LOAM Component
    loam::Odometry mapping_odometry_;
    // Imu messages as converted from the raw IMR file
    std::vector<loam::ImuMessage> imu_data_;
};

}  // namespace nie

#endif  // NIE_LOAM_LOAM_MANAGER_HPP
