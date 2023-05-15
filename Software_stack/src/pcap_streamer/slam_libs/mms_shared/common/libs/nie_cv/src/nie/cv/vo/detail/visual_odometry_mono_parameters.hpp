/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_MONO_PARAMETERS_HPP
#define NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_MONO_PARAMETERS_HPP

#include <Eigen/Dense>

#include "visual_odometry_mono_parameters.hpp"

namespace nie {

namespace detail {

// Mono VO parameters
struct VisualOdometryMonoParameters {
    explicit VisualOdometryMonoParameters(Eigen::Matrix3d K)
        : K(std::move(K)),
          keyframe_window_size(7),
          ba_window_size(6),
          ba_window_overlap(4),
          back_projection_error(0.5),
          global_flow_threshold(30.) {}

    Eigen::Matrix3d K;

    // Number of frames with significant change to be taken into account for motion estimation (> 1).
    // For example (when such check is used), when a frame is encountered with a small local flow, then this frame could
    // be discarded and so only useful frames will count in the keyframe window size.
    std::size_t keyframe_window_size;
    // Number of key frames to be considered for bundle adjustment (> 1)
    std::size_t ba_window_size;
    // Number of key frames overlapping in window of bundle adjustment
    std::size_t ba_window_overlap;

    // Back-projection error to determine outlier objects / matching chains in double pass bundle adjustment. When -1
    // no outlier removal is done and just a single pass bundle adjustment is performed.
    float back_projection_error;

    // The average global flow of the feature matches between key frames should be above this threshold in order for a
    // motion calculation to be done. Otherwise the current frame is skipped and the next frame will be considered as
    // keyframe.
    float global_flow_threshold;
};

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_MONO_PARAMETERS_HPP
