/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <vector>

// PCL includes
#include <pcl/point_types.h>

// NIE includes
#include <nie/core/time.hpp>

// Local includes
#include "lidar_packet_consumer.hpp"
#include "nie/lidar/io/streamer.hpp"
#include "ouster_calibration.hpp"
#include "ouster_packets.hpp"

namespace nie {

namespace io {

namespace ouster {

class PacketConsumer final : public lidar::LidarPacketConsumer<Packet> {
public:
    /**
     * \brief Constructor taking an optional path to the lidar calibration file.
     *
     * \param[in] lidar_calibration Laser calibration containing type specific information.
     */
    explicit PacketConsumer(LidarCalibration lidar_calibration);

    void ProcessPacket(std::unique_ptr<Packet> const& packet) override;

private:
    // TODO: consider using FixedAngle instead of encoder_count
    /// \brief Convert a single channel data to a single 3D point
    /// \param encoder_count lidar encoder count in the range [0, 90111] indicating the angle of the lidar
    /// \param channel_data_block single channel data block
    /// \param calibration calibration data of a single channel
    /// \return 3D point
    [[nodiscard]] inline pcl::PointXYZI ComputePoint(
            double encoder_count,
            ChannelDataBlock const& channel_data_block,
            LaserCalibration const& calibration) const;

    /// \brief Converts all lasers in a Lidar Data Packet to 3D points and call the installed callbacks
    /// \tparam channels how many lasers channels are available. Possible values: {32, 64, 128}
    /// \param packet Ouster Lidar Data packet
    template <std::size_t channels>
    void ProcessLidarDataPacket(LidarDataPacket<channels> const& packet);

    LidarCalibration lidar_calibration_;

    // Used to check if all the points from a given frame_id were already added to a pointcloud
    // Note that this refers to Ouster's azimuth block format (not to be confused with ROS' or PCL's concepts of
    // frame_id)
    std::uint16_t previous_azimuth_block_frame_id_{};

    // The frame id's from Ouster lidars cycle through uint16 values. So, no matter which value
    // previous_azimuth_block_frame_id_ is initialized to, it's very likely that the first frame id read from a packet
    // will be different than that value. This will trigger a harmless but undesired invocation of a sweep callback that
    // would return no points. Thus we use this boolean to fix this situation.
    bool is_first_run_{true};
};

}  // namespace ouster

}  // namespace io

}  // namespace nie
