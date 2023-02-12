/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_STATE_HPP
#define NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_STATE_HPP

#include <glog/logging.h>

#include "visual_odometry_mono_parameters.hpp"

namespace nie {

namespace detail {

// Mono VO state tracking object
class VisualOdometryState {
public:
    explicit VisualOdometryState(VisualOdometryMonoParameters const& parameters)
        : keyframe_counter_(parameters.keyframe_window_size - 1),
          init_ba_counter_(keyframe_counter_ * (parameters.ba_window_size - 1)),
          diff_ba_counter_(keyframe_counter_ * (parameters.ba_window_size - parameters.ba_window_overlap)),
          first_image_(true),
          first_motion_(true),
          first_ba_(true),
          counter_(0) {
        CHECK(parameters.keyframe_window_size > 1)
            << "For VisualOdometryMonoParameters, keyframe_window_size should be larger than 1.";
        CHECK(parameters.ba_window_size > 1)
            << "For VisualOdometryMonoParameters, ba_window_size should be larger than 1.";
        CHECK(parameters.ba_window_overlap < parameters.ba_window_size)
            << "For VisualOdometryMonoParameters, ba_window_overlap should be smaller than ba_window_size.";
    }

    bool IsFirstImage() const { return first_image_; }
    bool IsFirstMotion() const { return first_motion_; }

    bool IsKeyFrame() const { return counter_ % keyframe_counter_ == 0; }
    bool IsBaFrame() const { return counter_ == 0 && !first_image_; }

    // Should be called at the end of an iteration / processing an image
    void Next() {
        if (first_image_) {
            first_image_ = false;
        } else if (first_motion_ && !first_image_ && IsKeyFrame()) {
            first_motion_ = false;
        } else if (first_ba_ && !first_motion_ && IsBaFrame()) {
            first_ba_ = false;
        }

        ++counter_;
        // The only reason of doing the following reset is to not let the counter grow too large.
        if (counter_ == (first_ba_ ? init_ba_counter_ : diff_ba_counter_)) {
            counter_ = 0;
        }
    }

private:
    std::size_t const keyframe_counter_;
    std::size_t const init_ba_counter_;
    std::size_t const diff_ba_counter_;

    bool first_image_;
    bool first_motion_;
    bool first_ba_;

    std::size_t counter_;
};

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_STATE_HPP
