/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "loam_manager.hpp"

#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/formats/inertial_explorer/project_name.hpp>

#include "utils/imr_reader.hpp"

namespace {

std::vector<loam::ImuMessage> ImrDataToImuMessages(loam::ImrData const& imr_data, std::string const& project_name) {
    std::vector<loam::ImuMessage> ret;

    auto date_from_project_name = nie::io::inertial_explorer::ParseDateFromProjectName(project_name);
    auto const imu_epoch_from_project_name =
            nie::ToGPSWeekTime(nie::ToGPSTime(
                                       date_from_project_name,
                                       nie::io::inertial_explorer::kTimezoneOffset,
                                       false,
                                       nie::RepresentDoubleAsDuration<std::chrono::nanoseconds>(0.0)))
                    .week;

    double const accel_scale = imr_data.header.dAccelScaleFactor * imr_data.header.dDataRateHz;
    double const gyro_scale = imr_data.header.dGyroScaleFactor * imr_data.header.dDataRateHz;

    for (auto const& record : imr_data.records) {
        nie::GPSWeekTime<std::chrono::nanoseconds> timestamp{};
        timestamp.week = imu_epoch_from_project_name;
        timestamp.time_in_week = nie::RepresentDoubleAsDuration<std::chrono::nanoseconds>(record.Time);
        loam::ImuMessage imu_measurement;
        imu_measurement.header.seq = ret.size();
        imu_measurement.header.stamp = nie::ToGPSTime(timestamp);
        imu_measurement.orientation = decltype(imu_measurement.orientation)::Identity();
        imu_measurement.angular_velocity =
                Eigen::Vector3d(nie::Deg2Rad(record.gx), nie::Deg2Rad(record.gy), nie::Deg2Rad(record.gz)) * gyro_scale;
        imu_measurement.linear_acceleration = Eigen::Vector3d(record.ax, record.ay, record.az) * accel_scale;
        ret.push_back(std::move(imu_measurement));
    }

    return ret;
}

}  // namespace

namespace nie {

LoamManager::LoamManager(
        loam::MultiScanMapper const& multi_scan_mapper,
        loam::RegistrationParams const& config,
        std::string const& imr_file_path)
    : multi_scan_registration_(multi_scan_mapper) {
    multi_scan_registration_.configure(config);

    if (!imr_file_path.empty()) {
        auto imr_data = loam::ReadImrCollection(imr_file_path);
        imu_data_ =
                ImrDataToImuMessages(imr_data, boost::filesystem::path(imr_file_path).stem().string().substr(0, 14));
    }
}

void LoamManager::ProcessLidarReturns(nie::io::lidar::Returns const& lidar_returns) {
    nie::Timestamp_ns lidar_sweep_stamp = lidar_returns.timestamps.front();

    // Default constructor of ImuMessage is an "empty" but valid message
    loam::ImuMessage imu_measurement{};
    loam::ImuMessage imu_measurement_prev{};
    if (!imu_data_.empty()) {
        // Query closest imu message before the current point cloud. Using a
        // dummy ImuMessage that only really has a timestamp.
        loam::ImuMessage imu_query;
        imu_query.header.stamp = lidar_sweep_stamp;
        auto imu_lower_bound_it = std::lower_bound(
                imu_data_.cbegin(),
                imu_data_.cend(),
                imu_query,
                [](loam::ImuMessage const& lhs, loam::ImuMessage const& rhs) {
                    return lhs.header.stamp < rhs.header.stamp;
                });

        if (imu_lower_bound_it != imu_data_.end()) {
            imu_measurement = *imu_lower_bound_it;
            if (imu_lower_bound_it != imu_data_.cbegin()) {
                imu_measurement_prev = *(imu_lower_bound_it - 1);
            }  // else: is identity
        }      // else identity imu transformations are applied
    }

    // Use IMU measurement and point cloud for registration
    if (seq_ == 0 && !imu_data_.empty()) {
        LOG(INFO) << "Boundary condition: Feeding one extra IMU message before processing the first lidar sweep.";
        multi_scan_registration_.handleIMUMessage(imu_measurement_prev);
    }

    if (!imu_data_.empty()) {
        multi_scan_registration_.handleIMUMessage(imu_measurement);
    }
    multi_scan_registration_.handleCloudMessage(detail::LidarReturnsToPointCloudMessage(lidar_returns, seq_++));

    // Run lidar odometry
    laser_odometry_.imuTransHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.imuTransform(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.laserCloudFullResHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.laserCloud(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.laserCloudSharpHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.cornerPointsSharp(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.laserCloudLessSharpHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.cornerPointsLessSharp(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.laserCloudFlatHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.surfacePointsFlat(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.laserCloudLessFlatHandler(detail::CreatePointCloudMessage(
            multi_scan_registration_.surfacePointsLessFlat(), lidar_sweep_stamp, seq_, "lidar_frame"));
    laser_odometry_.process();

    // Run lidar mapping
    if (!imu_data_.empty()) {
        laser_mapping_.imuHandler(imu_measurement);
    }
    laser_mapping_.laserCloudCornerLastHandler(
            detail::CreatePointCloudMessage(*laser_odometry_.lastCornerCloud(), lidar_sweep_stamp, seq_, "odom_frame"));
    laser_mapping_.laserCloudSurfLastHandler(detail::CreatePointCloudMessage(
            *laser_odometry_.lastSurfaceCloud(), lidar_sweep_stamp, seq_, "odom_frame"));
    laser_mapping_.laserCloudFullResHandler(
            detail::CreatePointCloudMessage(*laser_odometry_.laserCloud(), lidar_sweep_stamp, seq_, "odom_frame"));
    laser_mapping_.laserOdometryHandler(laser_odometry_.laserOdometry());
    laser_mapping_.process();

    mapping_odometry_ = laser_mapping_.odomAftMapped();
    if (!laser_mapping_.laserCloud().empty()) {
        mapping_laser_cloud_ += laser_mapping_.laserCloud();
    }

    // Run transform maintenance
    transform_maintenance_.laserOdometryHandler(laser_odometry_.laserOdometry());
    transform_maintenance_.odomAftMappedHandler(laser_mapping_.odomAftMapped());
}

}  // namespace nie
