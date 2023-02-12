/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <string>
#include <vector>

// PCL includes
#include <pcl/point_types.h>

// NIE includes
#include <nie/core/time.hpp>

// Local includes
#include "lidar_packet_consumer.hpp"
#include "nie/lidar/io/streamer.hpp"
#include "velodyne_calibration.hpp"
#include "velodyne_packets.hpp"

namespace nie {

namespace io {

namespace velodyne {

// TODO: [EDD]
//             * Optional time / azimuth corrections within a block
//                  --> Currently always on
//             * Optional output of partial sweeps (first sweep, last sweep, or due to lost packets)
//                  --> Currently first / last sweep not output, packets loss undefined
//             * Optional dual (or multiple) returns
//                  --> Currently not implemented
/**
 * \brief PacketConsumer takes packets from a LiDAR and produces point clouds.
 *
 * There are two callbacks that return the same kind of data, but they are called at different rates, and have different
 * guarantees, and expected amount of points:
 *
 *  - kSweep   should be used when entire sweeps are required. A sweep is defined to be in the range [0...360>
 *                  degrees. It is undefined if a sweep callback contains a full sweeps when UDP packets are lost,
 *                  duplicated, or received out-of-order.
 *
 *  - kPacket  should be used if you want to receive data as-soon-as-possible, and / or if you are interested
 *                  in partial of sweeps.
 */
class PacketConsumer final : public lidar::LidarPacketConsumer<Packet> {
public:
    /**
     * \brief Constructor taking an optional path to the lidar calibration file.
     *
     * \param[in] lidar_calibration Laser calibration containing type specific information.
     */
    explicit PacketConsumer(LidarCalibration lidar_calibration);

    /// For Velodyne devices a sweep callback is generated when the azimuth of a lidar return is smaller than the
    /// azimuth of the previous beam (Effectively: When it passes 0 degrees). This happens at 5-20Hz (in device time)
    /// depending on the configuration of the Velodyne device. Currently, the first and last sweep do not generate a
    /// callback, because they are likely to be incomplete anyway.
    void ProcessPacket(std::unique_ptr<Packet> const& packet) override;

private:
    // Whenever the laser azimuth wraps back to zero degrees a new sweep begins.
    // The previous sweep is only considered completed if the following conditions hold:
    //  - previous_azimuth_process_laser_packet_ > 360 - kSweepCompletedMarginDegrees
    //  - first_azimuth_in_sweep_process_laser_packet_ < kSweepCompletedMarginDegrees
    constexpr static double kSweepCompletedMarginDegrees{1.0};

    [[nodiscard]] inline pcl::PointXYZI ComputePoint(
            FixedAngle<> const& azimuth, LaserReturn const& laser_return, LaserCalibration const& calibration) const;

    inline void CheckHourRollOver(std::chrono::microseconds const& current_packet_timestamp);

    void ProcessLaserPacket(LaserPacket const& packet);

    void ProcessTimePacket(TimePacket const& packet);

    double EstimateRotationSpeed(LaserPacket const& packet);

    // Sine and cosine lookup tables
    using LookUpTableSinCos = std::array<std::tuple<double, double>, FixedAngle<>::kTotalSteps>;

    static LookUpTableSinCos GenerateLookUpTableSinCos();
    static LookUpTableSinCos const kLookUpTableSinCos;

    // Flag that indicates if the GPS time is valid (e.g. Time is invalid at startup or when UDP anomalies occur)
    bool gps_time_valid_{false};

    // Extracted GPS time (in whole hours)
    Timestamp_ns gps_time_whole_hour_{};

    // Keep track of timestamp of previous packet to determine when gps_time_whole_hour should roll over
    std::chrono::microseconds previous_packet_timestamp_;

    // Type of lidar to be streamed
    LidarCalibration const lidar_calibration_;

    // Former static variables
    std::string previous_message_{};
    std::uint64_t previous_time_since_hour_{};
    FixedAngle<> previous_azimuth_estimate_rotation_speed_{};
    double previous_rotation_speed_{};
    std::uint8_t init_packets_{1};
    FixedAngle<> previous_azimuth_process_laser_packet_{};
    FixedAngle<> first_azimuth_in_sweep_process_laser_packet_{};
};

}  // namespace velodyne

}  // namespace io

}  // namespace nie
