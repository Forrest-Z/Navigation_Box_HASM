/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "ouster_packet_consumer.hpp"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <utility>

// Boost includes
#include <boost/math/constants/constants.hpp>

// Google includes
#include <glog/logging.h>

// PCL includes
#include <pcl/point_cloud.h>

// Local includes
#include "ouster_packets.hpp"

namespace nie {

namespace io {

namespace ouster {

PacketConsumer::PacketConsumer(LidarCalibration lidar_calibration)
    : lidar::LidarPacketConsumer<Packet>{}, lidar_calibration_{std::move(lidar_calibration)} {}

void PacketConsumer::ProcessPacket(std::unique_ptr<Packet> const& packet) {
    switch (packet->size()) {
        case sizeof(LidarDataPacket<32>):
            ProcessLidarDataPacket(*reinterpret_cast<LidarDataPacket<32>*>(packet->data()));
            break;
        case sizeof(LidarDataPacket<64>):
            ProcessLidarDataPacket(*reinterpret_cast<LidarDataPacket<64>*>(packet->data()));
            break;
        case sizeof(LidarDataPacket<128>):
            ProcessLidarDataPacket(*reinterpret_cast<LidarDataPacket<128>*>(packet->data()));
            break;
        case 48:
            // TODO: Ouster lidars also sends IMU packets of 48 bytes. Deal with them too.
            break;
        default:
            LOG(WARNING) << "Unexpected packet with size = " << packet->size();
            break;
    }
}

template <std::size_t channels>
void PacketConsumer::ProcessLidarDataPacket(LidarDataPacket<channels> const& packet) {
    pcl::PointCloud<pcl::PointXYZI> packet_points;
    std::vector<Timestamp_ns> packet_timestamps;

    // Initialize only if there is a callback installed
    if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
        static constexpr std::size_t kLasersPerPacket = kAzimuthBlocksPerPacket * channels;
        // Reserve maximum amount of points
        packet_points.reserve(kLasersPerPacket);
        // TODO: consider keeping track of the timestamp per encoder angle instead of per point
        packet_timestamps.reserve(kLasersPerPacket);
    }

    // Valid azimuth blocks *always* have their status set to 0xFFFFFFFF if they contain valid data in its channels'
    // Data Blocks or 0x0 otherwise
    static std::uint32_t constexpr kValidAzimuthBlockStatus = 0xFFFFFFFF;

    for (auto const& azimuth_block : packet.azimuth_blocks) {
        if (azimuth_block.block_status != kValidAzimuthBlockStatus) {
            LOG(WARNING) << "Azimuth block does not have valid data. Skipping.";
            continue;
        }

        if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>() &&
            azimuth_block.frame_id != previous_azimuth_block_frame_id_ && !is_first_run_) {
            sweep_points_.header.seq = sweep_id_++;
            callbacks_.Callback<lidar::LidarCallbackTags::kSweep>(
                    lidar::Returns{sweep_points_, std::move(sweep_timestamps_)});
            // Clear vectors
            sweep_points_ = {};
            sweep_timestamps_ = std::vector<Timestamp_ns>{};
        }
        for (std::size_t i = 0; i < azimuth_block.channel_data_blocks.size(); i++) {
            // Convenience aliases
            auto const& channel_data_block = azimuth_block.channel_data_blocks[i];
            auto const& beam_altitude_radians = lidar_calibration_.beam_altitude_radians[i];
            auto const& beam_azimuth_radians = lidar_calibration_.beam_azimuth_radians[i];

            auto const point = ComputePoint(
                    azimuth_block.encoder_count, channel_data_block, {beam_altitude_radians, beam_azimuth_radians});

            // Skip if the coordinate is not valid
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                VLOG(5) << "Invalid coordinate = " << point << ". Skipping.";
                continue;
            }

            if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
                packet_points.push_back(point);
                packet_timestamps.emplace_back(std::chrono::nanoseconds{azimuth_block.timestamp});
            }

            if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>()) {
                sweep_points_.push_back(point);
                sweep_timestamps_.emplace_back(std::chrono::nanoseconds{azimuth_block.timestamp});
            }
        }
        previous_azimuth_block_frame_id_ = azimuth_block.frame_id;
    }

    if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
        packet_points.header.seq = packet_id_++;
        callbacks_.Callback<lidar::LidarCallbackTags::kPacket>(
                lidar::Returns{std::move(packet_points), std::move(packet_timestamps)});
    }

    is_first_run_ = false;
}

template <typename To, typename From>
static inline To MmToMeters(From mm) {
    return static_cast<To>(mm * 1.0e-3);
}

pcl::PointXYZI PacketConsumer::ComputePoint(
        double const encoder_count,
        ChannelDataBlock const& channel_data_block,
        LaserCalibration const& calibration) const {
    // Calculations based on section 3.1 of the Software User Manual as of Jan 15, 2021
    // Throughout this function we refer to encoder coordinate system as 'sensor coordinate frame' and the lidar
    // coordinate system as 'lidar coordinate frame`

    // Referred to as 'r' in the Software User Manual
    double const range_mm{static_cast<double>(channel_data_block.range)};

    // Check if point should be ignored because minimum / maximum distance
    double const range_meters{MmToMeters<double>(range_mm)};
    if (range_meters < distance_thresholds_.min || range_meters > distance_thresholds_.max) {
        return lidar::MakeNanPoint();
    }

    // As defined in the Software User Manual in Chapter 3.1, the lidar coordinate frame is different from our
    // orientation convention for the LiDAR output. While it does follow the Right Hand Rule convention, it has the x
    // pointing towards the connector on the "back" of the sensor. This is different from the other LiDAR sensors.
    // They state that their drivers and visualization tools do convert this from their "Lidar Coordinate Frame" to the
    // "Sensor Coordinate Frame". The latter does agree with our convention.
    // To replicate this, we simply add PI to the azimuth, which rotates the coordinate frame 180 degrees around the z
    // axis.
    static double constexpr kThetaToSensorConvention{boost::math::double_constants::pi};
    static double constexpr kEncoderCountMax{90112.0};
    // Referred to as 'n' in the User Guide
    double const lidar_origin_to_beam_origin_mm{lidar_calibration_.lidar_origin_to_beam_origin_mm};
    // Horizontal angle of the sensor w.r.t. the lidar origin
    // It is negative since the encoder rotates anti-clockwise w.r.t to the lidar origin and the lidar frame has Z
    // pointing up
    double const theta_encoder_rad{-boost::math::double_constants::two_pi * encoder_count / kEncoderCountMax};

    // Horizontal angle of the beam w.r.t. the sensor origin
    // It is negative since the lidar frame has Z pointing up
    // For more information, see section 3.1.2 of the User Guide
    double const theta_azimuth_rad{-calibration.beam_azimuth_radians};
    // Vertical angle of the beam w.r.t. the sensor origin
    double const phi_rad{calibration.beam_altitude_radians};

    // Reused values
    // The range distance consists of two straight lines at an angle, broken by the encoder. This encoder is the beam
    // origin. The distance from the origin of the lidar to the encoder is described as "n" or
    // "lidar_origin_to_beam_origin". To determine the final point, we have to deal with these distances separately.
    // There are figures illustrating this situation in the User Guide in section 3.1.2 (figures 3.1 and 3.2)
    double const range_from_beam_origin_mm{range_mm - lidar_origin_to_beam_origin_mm};
    double const sum_thetas_rad{theta_encoder_rad + theta_azimuth_rad + kThetaToSensorConvention};
    double const cos_phi{std::cos(phi_rad)};

    // Main calculation
    // TODO: make use of lookup tables for cos/sin if FixedAngle is used
    double const x{
            range_from_beam_origin_mm * std::cos(sum_thetas_rad) * cos_phi +
            lidar_origin_to_beam_origin_mm * std::cos(theta_encoder_rad)};
    double const y{
            range_from_beam_origin_mm * std::sin(sum_thetas_rad) * cos_phi +
            lidar_origin_to_beam_origin_mm * std::sin(theta_encoder_rad)};
    double const z{range_from_beam_origin_mm * std::sin(phi_rad)};

    // Return point in meters
    pcl::PointXYZI point;
    // TODO: also add ambient/reflectivity data
    point.intensity = static_cast<float>(channel_data_block.signal_photons);
    point.x = MmToMeters<float>(x);
    point.y = MmToMeters<float>(y);
    point.z = MmToMeters<float>(z);
    return point;
}

}  // namespace ouster

}  // namespace io

}  // namespace nie