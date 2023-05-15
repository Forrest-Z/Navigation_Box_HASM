/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <numeric>

#include <pcl/point_cloud.h>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "lidar_point_interpolator.hpp"
#include "nie/lidar/io/lidar_streamer.hpp"

namespace nie {

/// This class is responsible for accumulating lidar data and related poses up to a certain point. Once a certain motion
/// threshold is reached, this data is forwarded via a callback function.
template <typename PointT>
class LidarStreamConsumer {
public:
    using SliceCallback = std::function<void(
            pcl::PointCloud<PointT> const&, Eigen::Vector3d const&, std::vector<Timestamp_ns> const&)>;

    LidarStreamConsumer(double const slice_size, SliceCallback const slice_callback)
        : slice_callback_{slice_callback},
          previous_isometry_{},
          current_isometry_{},
          lidar_returns_{},
          lidar_poses_{},
          slicing_delta_{},
          slice_size_{slice_size},
          has_previous_{false} {
        CHECK(slice_callback_) << "A slice callback should be set.";
    }

    void operator()(io::lidar::Returns points, io::PoseRecord pose) {
        // Check if the sweeps can be flushed
        if (has_previous_) {
            // Swap previous and current.
            previous_isometry_ = current_isometry_;
            current_isometry_ = pose.isometry;
        } else {
            // First entry. Delta = 0.
            current_isometry_ = pose.isometry;
            previous_isometry_ = current_isometry_;
            has_previous_ = true;
        }
        slicing_delta_ += previous_isometry_.TransformInverseLeft(current_isometry_).translation().norm();

        if (slicing_delta_ >= slice_size_) {
            // The last pose needs to be added before the flush is done, otherwise only the first point of the last
            // sweep will be written. Because, in that case, the rest of the points of the last sweep would have a
            // timestamp past the last pose, therefore no interpolation can be performed and so the points will be
            // discarded.
            lidar_poses_.emplace_back(pose);
            FlushSlice();

            slicing_delta_ = 0.0;
            // Reset the accumulation containers
            lidar_returns_.clear();
            lidar_poses_.clear();
        }

        // Accumulate the data
        lidar_returns_.emplace_back(std::move(points));
        lidar_poses_.emplace_back(std::move(pose));
    }

    void Stop() {
        VLOG(1) << "Lidar stream consumer is stopped.";
        // Making sure there is (enough) data to write
        if (slicing_delta_ > 0.0 && lidar_poses_.size() > 1) {
            VLOG(2) << "Writing trailing data.";
            FlushSlice();
        }
    }

private:
    void FlushSlice() {
        // Aggregate all lidar returns
        // Note that an original return has a cloud with N points and N timestamp. This return is accompanied with one
        // pose. In the accumulated containers this information containers this information is lost as the sizes will be
        // (M = #returns, N = #average points/sweep):
        //   * returns:    M
        //   * points:     M * N
        //   * timestamps: M * N
        //   * poses:      M
        // Another note, the aggregation is not done on a per lidar return basis, because the accumulation containers
        // would then have to be resized/reserved for every lidar return. This would be expensive to do. This will
        // happen mainly the first time and the times after that when the container needs to be even larger. (The
        // original memory will be kept, capacity of vector remains even when the size is 0 after clearing.) The price
        // is that for every slice the data is copied at least one. the original vector and the two resulting vectors.
        // As mainly smaller data sets are processed at the moment, this is good, but for larger data sets, this could
        // be something to improve in future.
        pcl::PointCloud<PointT> points;
        std::vector<Timestamp_ns> timestamps;
        AggregateReturns(lidar_returns_, &points, &timestamps);

        // Point coordinates in the pcl cloud are stored as floats. As the transformation of the points to the pose
        // system can lead to large coordinates, precision might be lost. Therefore an offset is taken in to account.
        Eigen::Vector3d const offset = lidar_poses_[lidar_poses_.size() / 2].isometry.translation();

        // Transform all lidar points to the world
        LidarPointInterpolator::TransformPoints(lidar_poses_, offset, &points, &timestamps);

        // Note that the bounds that are calculated do not take the offset into account.
        slice_callback_(points, offset, timestamps);
    }

    // This function combines all the individual lidar returns.
    static void AggregateReturns(
            std::vector<io::lidar::Returns> const& returns,
            pcl::PointCloud<PointT>* p_points,
            std::vector<Timestamp_ns>* p_timestamps) {
        pcl::PointCloud<PointT>& points = *p_points;
        std::vector<Timestamp_ns>& timestamps = *p_timestamps;

        std::size_t const total_size = std::accumulate(
                returns.cbegin(), returns.cend(), 0, [](std::size_t const sum, io::lidar::Returns const& r) {
                    return sum + r.timestamps.size();
                });
        points.reserve(total_size);
        timestamps.reserve(total_size);
        std::for_each(returns.cbegin(), returns.cend(), [&points, &timestamps](io::lidar::Returns const& r) {
            points += r.points;
            std::copy(r.timestamps.begin(), r.timestamps.end(), std::back_inserter(timestamps));
        });
    }

    SliceCallback const slice_callback_;

    Isometry3qd previous_isometry_;
    Isometry3qd current_isometry_;

    // Accumulation containers
    std::vector<io::lidar::Returns> lidar_returns_;
    std::vector<io::PoseRecord> lidar_poses_;

    // Process lidar returns and returns up to a certain size (in meters). Then
    // flush the processors to obtain some output.
    double slicing_delta_;
    double const slice_size_;
    // Initialization sentinel to account for the fact that we need two entries
    // to compute a delta.
    bool has_previous_;
};

}  // namespace nie
