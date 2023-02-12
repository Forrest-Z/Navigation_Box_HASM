/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// Standard C++ includes
#include <string>
#include <vector>

// PCL includes
#include <pcl/point_types.h>

// NIE includes
#include <nie/core/time.hpp>

// Local includes
#include "kitti_packets.hpp"
#include "lidar_packet_consumer.hpp"
#include "nie/lidar/io/streamer.hpp"

namespace nie {

namespace io {

namespace kitti {

class PacketConsumer final : public lidar::LidarPacketConsumer<Packet> {
public:
    void ProcessPacket(std::unique_ptr<Packet> const& packet) override {
        CHECK(!callbacks_.HasCallback<lidar::LidarCallbackTags::kPacket>() &&
              !callbacks_.HasCallback<lidar::LidarCallbackTags::kSweepAndAngles>())
                << "Callbacks for packet and sweep with angles not supported for kitti data.";

        if (callbacks_.HasCallback<lidar::LidarCallbackTags::kSweep>()) {
            std::vector<Timestamp_ns> timestamps =
                    GenerateTimestamps(packet->time_begin, packet->time_end, packet->points.size());
            callbacks_.Callback<lidar::LidarCallbackTags::kSweep>(
                    lidar::Returns{std::move(packet->points), std::move(timestamps)});
        }
    }

private:
    static std::vector<Timestamp_ns> GenerateTimestamps(
            Timestamp_ns const& begin, Timestamp_ns const& end, std::size_t const size) {
        // The following calculation has to be done using a counter and a double typed step value, otherwise rounding
        // effects are too large (250000 points * 0.4 ns rounding in step size = 0.1 ms mismatch of end time)
        std::vector<Timestamp_ns> result(size);
        auto const step = std::chrono::duration<double, std::nano>(end - begin) / size;
        std::generate(result.begin(), result.end(), [index = 0, &begin, &step]() mutable {
            return begin + std::chrono::duration_cast<std::chrono::nanoseconds>(step * index++);
        });
        return result;
    }
};

}  // namespace kitti

}  // namespace io

}  // namespace nie
