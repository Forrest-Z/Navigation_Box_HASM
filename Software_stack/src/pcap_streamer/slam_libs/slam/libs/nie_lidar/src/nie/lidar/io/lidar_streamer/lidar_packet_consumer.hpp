/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/core/callbacks.hpp>
#include <nie/core/time.hpp>

#include "lidar_data_types.hpp"

namespace nie {

namespace io {

namespace lidar {

// TODO: [EDD] When the lidar is synchronised with the GPS and its PPS (pulse per second), then it also generates
//             raw GPS positions (lat / lon / speed / course / magnetic_var). We could add a callback for these values.
// TODO: [EDD] According to the HDL-32E documentation that device incorporates three orthogonal MEMS gyroscopes,
//             three orthogonal MEMS accelerometers and three temperature sensors. We could add a callback for these
//             values.
enum class LidarCallbackTags : std::size_t { kPacket, kSweep, kSweepAndAngles };

struct DistanceThresholds {
    /// Discard returns with a distance less than the given threshold [in m]
    float min{0.0f};
    /// Discard returns with a distance greater than the given threshold [in m]
    float max{std::numeric_limits<float>::max()};
};

inline auto MakeNanPoint() {
    pcl::PointXYZI point;
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    return point;
}

template <typename Packet>
class LidarPacketConsumer {
public:
    using CallbackTags = LidarCallbackTags;

    /// \brief Identical to Callback::AddCallback(), but enforces that the Streamer is not running
    template <CallbackTags index, typename Function>
    void AddCallback(Function function) {
        callbacks_.AddCallback<index, Function>(function);
    }

    /// This method should be implemented by a class that implements PacketHandler. It should receive a binary
    /// packet coming from a lidar device and then convert the contents of the packet to a point cloud and invoke the
    /// appropriate callback according to the mode (LidarCallbackTags::kSweep or LidarCallbackTags::kPacket)
    /// \param packet packet containing binary lidar data
    virtual void ProcessPacket(std::unique_ptr<Packet> const& packet) = 0;

    void SetDistanceThreshold(lidar::DistanceThresholds const& d) { distance_thresholds_ = d; }

protected:
    LidarPacketConsumer() = default;

    using CallbacksType = Callbacks<
            LidarCallbackTags,
            std::function<void(Returns)>,
            std::function<void(Returns)>,
            std::function<void(Returns, Angles)>>;

    CallbacksType callbacks_;

    std::size_t sweep_id_{0};
    std::size_t packet_id_{0};

    pcl::PointCloud<pcl::PointXYZI> sweep_points_;
    std::vector<Timestamp_ns> sweep_timestamps_{};

    lidar::Angles sweep_angles_{};

    /// The LidarPacketConsumer is a logical place to perform distance filtering because lidar packets being processed

    /// provide distance
    DistanceThresholds distance_thresholds_{};
};

}  // namespace lidar

}  // namespace io

}  // namespace nie
