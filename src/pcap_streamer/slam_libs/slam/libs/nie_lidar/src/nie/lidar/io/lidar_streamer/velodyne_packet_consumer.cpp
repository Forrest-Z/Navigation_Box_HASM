/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "velodyne_packet_consumer.hpp"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <utility>

// Google includes
#include <glog/logging.h>

// PCL includes
#include <pcl/point_cloud.h>

// NIE includes
#include <nie/core/geometry/rotation.hpp>

namespace nie {

namespace io {

namespace velodyne {

PacketConsumer::LookUpTableSinCos const PacketConsumer::kLookUpTableSinCos =
        PacketConsumer::GenerateLookUpTableSinCos();

PacketConsumer::LookUpTableSinCos PacketConsumer::GenerateLookUpTableSinCos() {
    // Fill sine and cosine lookup table
    // NOTE: Usage of lookup tables delivers a significant speedup
    //       These values can be precomputed without loss of precision because the reported device azimuth angle is
    //       is always reported in fixed angle steps (In our case 1/100 of a degree).
    LookUpTableSinCos lookup_table;

    for (std::size_t i = 0; i < lookup_table.size(); ++i) {
        double const angle = Deg2Rad(FixedAngle(i).Degrees());
        lookup_table[i] = {std::sin(angle), std::cos(angle)};
    }

    return lookup_table;
}

PacketConsumer::PacketConsumer(LidarCalibration lidar_calibration)
    : lidar::LidarPacketConsumer<Packet>{},
      previous_packet_timestamp_{std::numeric_limits<std::chrono::microseconds>::max()},
      lidar_calibration_{std::move(lidar_calibration)} {}

void PacketConsumer::CheckHourRollOver(std::chrono::microseconds const& current_packet_timestamp) {
    // Detect GPS hour roll over by comparing the current packet timestamp with the last packet timestamp. In theory,
    // if the current timestamp is smaller than the last timestamp, then the GPS hour has rolled-over. However, if
    // incoming packets are not strictly ordered (which could occur with UDP) then this approach fails. To handle this
    // we require that there is at least 30 minutes time difference between the current and last timestamp.
    if (gps_time_valid_ && previous_packet_timestamp_ - current_packet_timestamp > std::chrono::minutes(30)) {
        // Perform gps time roll over
        gps_time_whole_hour_ += std::chrono::hours(1);
    }

    // Store current packet timestamp
    previous_packet_timestamp_ = current_packet_timestamp;
}

void PacketConsumer::ProcessPacket(std::unique_ptr<Packet> const& packet) {
    // TODO: [EDD] Could compute the effective Baud rate
    switch (packet->size()) {
        case sizeof(TimePacket):
            ProcessTimePacket(*reinterpret_cast<TimePacket*>(packet->data()));
            break;

        case sizeof(LaserPacket):
            ProcessLaserPacket(*reinterpret_cast<LaserPacket*>(packet->data()));
            break;

        default:
            LOG(WARNING) << "Unexpected packet with size = " << packet->size();
    }
}

void PacketConsumer::ProcessTimePacket(TimePacket const& packet) {
    // Check if the GPS time should roll over
    std::chrono::microseconds const gps_time_since_hour{packet.gps_time_since_hour};
    CheckHourRollOver(gps_time_since_hour);

    // Extract NMEA message
    std::string const message(packet.nmea.cbegin(), packet.nmea.cend());

    // Ignore duplicate NMEA messages
    if (message == previous_message_) return;

    // Split NMEA message into parts
    auto const parts = Split<std::string>(message, ',');
    auto const& message_type = parts[0];  // $PG...

    // Verify NMEA message type is RMC (Recommended Minimum sentence C)
    if (message_type != "$GPRMC") {
        LOG(WARNING) << "Ignoring unexpected NMEA message type " << message_type;
        return;
    }

    // Assign to named variable (only for readability)
    // clang-format off
    auto const& utc_timestamp                 = parts[1];   // Format 'hhmmss.ss'
    auto const& validity                      = parts[2];   // A (valid), V (invalid)
//  auto const& latitude                      = parts[3];   // in degrees
//  auto const& latitude_hemisphere           = parts[4];   // N (north), S (south)
//  auto const& longitude                     = parts[5];   // in degrees
//  auto const& longitude_hemisphere          = parts[6];   // E (east), W (west)
//  auto const& speed_over_ground             = parts[7];   // in knots
//  auto const& true_course                   = parts[8];   // in degrees
    auto const& utc_datestamp                 = parts[9];   // Format 'ddmmyy'
//  auto const& magnetic_variation            = parts[10];  // in degrees
//  auto const& magnetic_variation_hemisphere = parts[11];  // E (east), W (west)
//  auto const& checksum                      = parts[12];
    // clang-format on

    // TODO: [EDD] Could verify NMEA checksum, see:
    //             https://en.wikipedia.org/wiki/NMEA_0183#C_implementation_of_checksum_generation

    // Verify NMEA message validity
    if (validity != "A") {
        return;
    }

    // Parse the UTC date and time strings
    std::chrono::year const year{std::stoi(utc_datestamp.substr(4, 2)) + 2000};
    std::chrono::month const month{stou(utc_datestamp.substr(2, 2))};
    std::chrono::day const day{stou(utc_datestamp.substr(0, 2))};
    std::chrono::hours const hours{stou(utc_timestamp.substr(0, 2))};

    // Verify that the UTC date is valid
    auto const utc_date = year / month / day;
    CHECK(utc_date.ok()) << "Unable to parse UTC date from NMEA message " << message;

    // Derive the UTC time that corresponds to the start of the hour, then convert it to GPS time
    auto const gps_time = std::chrono::clock_cast<std::chrono::gps_clock>(std::chrono::sys_days(utc_date) + hours);

    // Check if GPS time in this message is different from the stored gps time (This should happen only spuriously)
    if (gps_time_whole_hour_ != gps_time) {
        // Check if GPS time is currently considered to be valid
        if (gps_time_valid_) {
            LOG(WARNING) << "GPS time (in hours) updated from " << gps_time_whole_hour_ << " to " << gps_time;
        } else {
            LOG(INFO) << "GPS time (in hours) set to " << gps_time;
        }

        // Set new GPS time
        gps_time_whole_hour_ = gps_time;
        gps_time_valid_ = true;
    }

    // Store current NMEA message
    previous_message_ = message;
}

/**
 * Estimate device rotation speed from laser packet data
 *
 * Implements 2 methods to estimate the average lidar rotation speed:
 * 1) Use rotation angles and timestamps of 2 firings (as far apart as possible) from _a single laser packet_
 * 2) Use rotation angles and timestamps of 2 firings from _two consecutive laser packets_
 *
 * Method 1 has the advantage that it can be computed from each single packet (also for the first packet)
 * Method 2 is more stable because measurement and round-off errors are smoothed out
 *
 * @param packet  The last laser firing packet received from the packet producer
 *
 * @return Estimated rotation speed [in degrees / ns]
 */
// TODO: [EDD] Verify that the Streamer class should handle, and correctly handles UDP packets that are allowed to
//       be lost, duplicated, or delivered out-of-order. Check what the guarantees are that boost::asio and libpcap
//       give. If required, perhaps detection of UDP packet delivery anomalies can be detected at the IP / UDP level.
//       Note that at least estimation of rotation speed using method 2 will fail in case of these anomalies.
// TODO: [EDD] There is also a fundamental choice to be made; I have seen other implementations just skip sending a
//       lidar sweep if there are _no_ points detected, but for autonomous driving I would think that knowing that
//       nothing is detected is actually an important use-case.
double PacketConsumer::EstimateRotationSpeed(LaserPacket const& packet) {
    // Determine the number of microseconds in one hour
    constexpr std::uint64_t microseconds_per_hour = std::chrono::microseconds(std::chrono::hours(1)).count();

    // GPS time since the start of the hour [in microseconds] associated with the first laser return in the packet
    std::uint64_t current_time_since_hour = packet.gps_time_since_hour;

    // Azimuth associated with the first laser return in the packet
    FixedAngle current_azimuth = packet.blocks.cbegin()->azimuth;

    // Estimated rotation speed [in degrees / ns]
    double current_rotation_speed;

    // TODO: [EDD] Detect if there was packet loss, because then we need to reinitialise the algorithm
#if 0
    // Try to detect if there was packet loss using the current packet pace
    if (init_packets_ == 0) {

        // Reinitialise the algorithm
        init_packets_ = 1;
    }
#endif

    // Check if the algorithm needs to be initialised
    if (init_packets_ > 0) {
        // Estimate rotation speed from azimuth difference and elapsed time _within packet_ (less accurate)

        // Compute azimuth between start of first block and start of last block
        auto const azimuth_first_to_last_block = packet.blocks.crbegin()->azimuth - current_azimuth;

        // Compute average rotation speed from first to last block
        current_rotation_speed =
                azimuth_first_to_last_block.Degrees() / lidar_calibration_.time_first_to_last_block().count();

        // Decrement number of packets that require initialisation
        --init_packets_;
    } else {
        // Estimate rotation speed from azimuth difference and elapsed time _between packets_ (more accurate)

        // Compute time between packets (Account for wrapping over the hour)
        auto const time_between_packets =
                ((current_time_since_hour - previous_time_since_hour_ + microseconds_per_hour) % microseconds_per_hour);

        // Compute azimuth between packets (Wrapping of angle handled by FixedAngle class)
        auto const azimuth_between_packets = current_azimuth - previous_azimuth_estimate_rotation_speed_;

        // Compute rotation speed (Convert to nanoseconds)
        current_rotation_speed = azimuth_between_packets.Degrees() / static_cast<double>(time_between_packets * 1000);

        // Verify that the rotation speed is approximately constant (Warn if it changes more than 3%)
        const double rotation_speed_change =
                (current_rotation_speed - previous_rotation_speed_) / previous_rotation_speed_;
        if (std::abs(rotation_speed_change) > 0.03) {
            LOG(WARNING) << "The rotation speed of the Lidar is not constant! "
                         << "Change = " << rotation_speed_change * 100 << "% \t" << gps_time_whole_hour_;
        }
    }

    // Store current values for next packet
    previous_time_since_hour_ = current_time_since_hour;
    previous_azimuth_estimate_rotation_speed_ = current_azimuth;
    previous_rotation_speed_ = current_rotation_speed;

    return current_rotation_speed;
}

void PacketConsumer::ProcessLaserPacket(LaserPacket const& packet) {
    // Check if the GPS time is valid
    if (!gps_time_valid_) {
        DLOG(INFO) << "Skipping laser firing packet because the absolute GPS time is not yet known";
        return;
    }

    // Check if the gps time should roll over
    std::chrono::microseconds const gps_time_since_hour(packet.gps_time_since_hour);
    CheckHourRollOver(gps_time_since_hour);

    // Compute the GPS timestamp belonging to the first laser return in the packet
    Timestamp_ns gps_packet_time = gps_time_whole_hour_ + gps_time_since_hour;

    // Estimate rotation speed from (consecutive) packets [in degrees / ns]
    double const rotation_speed = EstimateRotationSpeed(packet);

    // Collect points and timestamps for lidar packet callbacks
    pcl::PointCloud<pcl::PointXYZI> packet_points;
    std::vector<Timestamp_ns> packet_timestamps;

    // Initialise only if there is a callback installed
    if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
        packet_points = pcl::PointCloud<pcl::PointXYZI>{};
        packet_timestamps = std::vector<Timestamp_ns>{};

        // Reserve maximum amount of points
        packet_points.reserve(kLasersPerPacket);
        packet_timestamps.reserve(kLasersPerPacket);
    }

    bool const collect_sweep = callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>() ||
                               callbacks_.HasCallback<lidar::LidarCallbackTags::kSweepAndAngles>();

    auto const time_between_block_firings = lidar_calibration_.time_between_block_firings();
    auto const time_between_laser_firings = lidar_calibration_.time_between_laser_firings();
    auto const num_lasers = lidar_calibration_.num_lasers();
    auto const laser_calibrations = lidar_calibration_.laser_calibrations();

    // Loop over all blocks in this packet
    for (std::uint8_t block_id = 0; block_id < kBlocksPerPacket; ++block_id) {
        // Obtain a reference to the block
        Block const& block = packet.blocks[block_id];

        // Use bit fiddling to derive laser id offset from block type:
        //   0xeeff -> Block 0  to 31  -> Offset 0
        //   0xddff -> Block 32 to 63  -> Offset 32
        //   0xccff -> Block 64 to 95  -> Offset 64
        //   0xbbff -> Block 96 to 127 -> Offset 96
        std::uint8_t const laser_id_offset = ((0xeeffu - block.type) >> 3u) & 0x60u;

        // Compute block time offset [in ns]
        auto const block_time_offset = time_between_block_firings * block_id;

        // Loop over all lasers in this block
        for (std::uint8_t laser_id = 0; laser_id < kLasersPerBlock; ++laser_id) {
            // Compute laser time offset [in ns]
            auto const laser_time_offset = time_between_laser_firings * laser_id;

            // Compute laser azimuth offset
            FixedAngle const laser_azimuth_offset{static_cast<std::uint16_t>(
                    std::round(rotation_speed * laser_time_offset.count() * FixedAngle<>::kStepsPerDegree))};

            // Compute corrected azimuth
            FixedAngle const laser_azimuth = block.azimuth + laser_azimuth_offset;

            // If the azimuth wraps then the sweep is finished
            if (collect_sweep && laser_azimuth < previous_azimuth_process_laser_packet_) {
                // Skip first packet
                // TODO: Only the angles of first point in sweep and last point in sweep are checked.
                // TODO: There might be gaps inside a sweep which also means that it is incomplete.
                if (previous_azimuth_process_laser_packet_.Valid() &&
                    first_azimuth_in_sweep_process_laser_packet_.Valid()) {
                    if (previous_azimuth_process_laser_packet_.Degrees() > 360 - kSweepCompletedMarginDegrees &&
                        first_azimuth_in_sweep_process_laser_packet_.Degrees() < kSweepCompletedMarginDegrees) {
                        // Set and increment the sweep id
                        sweep_points_.header.seq = sweep_id_++;

                        // Perform callbacks and pass the lidar sweep information avoiding copies as much as possible
                        if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>() &&
                            callbacks_.HasCallback<lidar::LidarCallbackTags::kSweepAndAngles>()) {
                            lidar::Returns returns{std::move(sweep_points_), std::move(sweep_timestamps_)};
                            callbacks_.Callback<lidar::LidarCallbackTags::kSweep>(returns);
                            callbacks_.Callback<lidar::LidarCallbackTags::kSweepAndAngles>(
                                    std::move(returns), std::move(sweep_angles_));
                        } else if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>()) {
                            callbacks_.Callback<lidar::LidarCallbackTags::kSweep>(
                                    lidar::Returns{std::move(sweep_points_), std::move(sweep_timestamps_)});
                        } else if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweepAndAngles>()) {
                            callbacks_.Callback<lidar::LidarCallbackTags::kSweepAndAngles>(
                                    lidar::Returns{std::move(sweep_points_), std::move(sweep_timestamps_)},
                                    std::move(sweep_angles_));
                        }
                    } else {
                        VLOG(3) << "Incomplete sweep skipped azimuths begin to end: "
                                << first_azimuth_in_sweep_process_laser_packet_.Degrees() << " to "
                                << previous_azimuth_process_laser_packet_.Degrees();
                    }
                } else {
                    VLOG(3) << "Invalid sweep skipped.";
                }

                first_azimuth_in_sweep_process_laser_packet_ = laser_azimuth;

                // Allocate new memory for points and timestamps
                sweep_points_ = {};
                sweep_timestamps_ = {};
                sweep_angles_.hor_angles = {};
                sweep_angles_.ver_angles = {};
            }

            // Store the current azimuth so we can check for wrapping
            previous_azimuth_process_laser_packet_ = laser_azimuth;

            // Get the index of the laser based on the current firing and number of lasers (e.g. VLP16 has 2 firing
            // sequences per block)
            uint laser_index = (laser_id + laser_id_offset) % num_lasers;
            // Compute the coordinate of this point in the device coordinate system
            auto const point =
                    ComputePoint(laser_azimuth, block.laser_returns[laser_id], laser_calibrations[laser_index]);

            // Skip if the coordinate is not valid
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                continue;
            }

            // Compute the timestamp for this laser return
            // TODO: [EDD] I found conflicting documentation of what the packet time means: Either the time of the first
            //       return or the time of the last return. Verify how VeloView interprets it
            auto const timestamp = gps_packet_time + block_time_offset + laser_time_offset;

            // Add point to lidar packet if there is at least one callback
            if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
                packet_points.push_back(point);
                packet_timestamps.push_back(timestamp);
            }

            // Add point to lidar sweep if there is at least one callback
            if (collect_sweep) {
                sweep_points_.push_back(point);
                sweep_timestamps_.push_back(timestamp);
                if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweepAndAngles>()) {
                    sweep_angles_.hor_angles.push_back(laser_azimuth.Degrees());
                    sweep_angles_.ver_angles.push_back(laser_calibrations[laser_index].vertical());
                }
            }
        }
    }

    if (callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>()) {
        // Set and increment the packet id
        packet_points.header.seq = packet_id_++;

        // Perform lidar packet callback
        callbacks_.Callback<lidar::LidarCallbackTags::kPacket>(
                lidar::Returns{std::move(packet_points), std::move(packet_timestamps)});
    }
}

pcl::PointXYZI PacketConsumer::ComputePoint(
        FixedAngle<> const& azimuth, LaserReturn const& laser_return, LaserCalibration const& calibration) const {
    // TODO: [EDD] Add intensity calibration
    // Create point with (uncalibrated) intensity
    pcl::PointXYZI point;
    point.intensity = static_cast<float>(laser_return.intensity);

    // Check if point should be ignored because minimum / maximum distance
    double const distance_in_meter = laser_return.distance * 0.002 + calibration.distance();

    // Check if point should be ignored because minimum / maximum distance
    if (distance_in_meter < distance_thresholds_.min || distance_in_meter > distance_thresholds_.max) {
        return lidar::MakeNanPoint();
    }

    // Obtain sine / cosine of the azimuth
    double sin_azimuth;
    double cos_azimuth;
    if (calibration.azimuth() == 0) {
        sin_azimuth = std::get<0>(kLookUpTableSinCos[azimuth.Steps()]);
        cos_azimuth = std::get<1>(kLookUpTableSinCos[azimuth.Steps()]);
    } else {
        double const azimuth_in_rad = Deg2Rad(azimuth.Degrees() - calibration.azimuth());
        sin_azimuth = std::sin(azimuth_in_rad);
        cos_azimuth = std::cos(azimuth_in_rad);
    }

    double const xy_distance = distance_in_meter * calibration.cos_vert();

    point.x = static_cast<float>(xy_distance * sin_azimuth - calibration.horz_offset() * cos_azimuth);
    point.y = static_cast<float>(xy_distance * cos_azimuth + calibration.horz_offset() * sin_azimuth);
    point.z = static_cast<float>(distance_in_meter * calibration.sin_vert() + calibration.vert_offset());

    if (point.x == 0 && point.y == 0 && point.z == 0) {
        point = lidar::MakeNanPoint();
    }

    return point;
}

}  // namespace velodyne

}  // namespace io

}  // namespace nie
