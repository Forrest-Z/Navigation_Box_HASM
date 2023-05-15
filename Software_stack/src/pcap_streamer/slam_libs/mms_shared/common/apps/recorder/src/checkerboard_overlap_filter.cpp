/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by pedro.raimundo on 14/02/19.
//

#include "checkerboard_overlap_filter.hpp"

#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "polygon_overlap.hpp"

namespace nie {
float const kCheckerboardDetectionScale = 0.5f;

CheckerboardOverlapFilter::CheckerboardOverlapFilter(cv::Size const& pattern_size, float overlap_threshold)
    : pattern_size_(pattern_size), overlap_threshold_(overlap_threshold) {}

// returns <has_checkerboard, is_accepted>
std::pair<bool, bool> CheckerboardOverlapFilter::Filter(cv::Mat const& current_frame) {
    std::vector<cv::Point2f> current_corners;
    cv::Mat gray;
    cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);
    // gotta do this to stay above 2FPS -- however, this limits the distance at
    // which an A3 checkerboard can be seen to about 2 meters
    cv::resize(gray, gray, cv::Size(), kCheckerboardDetectionScale, kCheckerboardDetectionScale);
    if (findChessboardCorners(
            gray, this->pattern_size_, current_corners, cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_ADAPTIVE_THRESH)) {
        // corners of the checkerboard parallelogram -- CCW
        std::vector<cv::Point2f> current_polygon{
            current_corners[0],
            current_corners[this->pattern_size_.width * (this->pattern_size_.height - 1)],
            current_corners[(this->pattern_size_.width * this->pattern_size_.height) - 1],
            current_corners[this->pattern_size_.width - 1]};

        double overlap_percentage = GetPolygonOverlapRatio(current_polygon, this->last_accepted_polygon_);

        VLOG(5) << "Overlap = " << overlap_percentage;

        if (overlap_percentage < this->overlap_threshold_) {
            this->last_accepted_polygon_ = current_polygon;
            return std::make_pair(true, true);
        } else {
            return std::make_pair(true, false);
        }
    }
    return std::make_pair(false, false);
}

}  // namespace nie