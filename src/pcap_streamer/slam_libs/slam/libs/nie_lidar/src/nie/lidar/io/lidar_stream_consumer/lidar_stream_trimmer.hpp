/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

namespace nie {

class LidarStreamTrimmer {
public:
    LidarStreamTrimmer(
            double const trimming_size,
            std::function<void(io::lidar::Returns const&, io::PoseRecord const&)> const sweep_callback)
        : sweep_callback_{sweep_callback},
          previous_isometry_{Isometry3qd::Identity()},
          current_isometry_{Isometry3qd::Identity()},
          has_previous_{false},
          startup_delta_{Eigen::Vector3d::Zero()},
          trimming_buffer_{},
          trimming_size_{trimming_size},
          trimming_size_squared_{trimming_size_ * trimming_size_} {
        CHECK(sweep_callback_) << "A sweep callback should be set.";
        CHECK(trimming_size_ > 0.0) << "Trim size should be larger than zero.";
    }

    void operator()(io::lidar::Returns lidar_points, io::PoseRecord pose) {
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

        // Skip first trimming_size_ meters of data. Note that this takes direction into consideration, such that going
        // back and forth (i.e. because of jitter and noise does not count.
        startup_delta_ += previous_isometry_.TransformInverseLeft(current_isometry_).translation();
        if (startup_delta_.norm() < trimming_size_) {
            return;
        }

        trimming_buffer_.emplace_back(std::move(lidar_points), std::move(pose));

        // Keep the last trimming_size_ meters of data buffered and forward the ones before that end section.
        while (trimming_buffer_.front()
                       .second.isometry.TransformInverseLeft(trimming_buffer_.back().second.isometry)
                       .translation()
                       .squaredNorm() > trimming_size_squared_) {
            sweep_callback_(trimming_buffer_.front().first, trimming_buffer_.front().second);
            trimming_buffer_.pop_front();
        }
    }

private:
    std::function<void(io::lidar::Returns const&, io::PoseRecord const&)> const sweep_callback_;

    Isometry3qd previous_isometry_;
    Isometry3qd current_isometry_;
    // Initialization sentinel to account for the fact that we need two entries to compute a delta.
    bool has_previous_;

    // Members related to the double-buffering logic necessary to trim the ends of the lidar data (plus related stuff).
    Eigen::Vector3d startup_delta_;
    std::deque<std::pair<io::lidar::Returns, io::PoseRecord>> trimming_buffer_;
    double const trimming_size_;
    double const trimming_size_squared_;
};

}  // namespace nie
