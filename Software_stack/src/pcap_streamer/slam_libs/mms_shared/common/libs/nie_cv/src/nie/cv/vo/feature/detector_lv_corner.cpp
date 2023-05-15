/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "detector_lv_corner.hpp"

#include <nie/core/scoped_timer.hpp>

#include <opencv2/opencv.hpp>

namespace nie {

void DetectorLvCorner::Detect(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) {
    ScopedTimer timer("DetectorLvCorner::Detect");

    cv::Mat const filter = Filter(image);
    FindFeatures(filter, p_features, p_feature_types);

    Callback<Handle::kFilter>(filter, *p_features);
}

cv::Mat DetectorLvCorner::Filter(cv::Mat const& image) {
    ScopedTimer timer("DetectorLvCorner::Filter");
    /*
     * The full filter looks like:
     *   |-1 -1  0  1  1|
     *   |-1 -1  0  1  1|
     *   | 0  0  0  0  0|
     *   | 1  1  0 -1 -1|
     *   | 1  1  0 -1 -1|
     * which can be separated like in two one dimensional filter like:
     *   | 1|
     *   | 1|
     *   | 0| * |-1 -1 0 1 1|
     *   |-1|
     *   |-1|
     */

    // Make sure filter image can fit the calculated values
    int new_depth = IncreaseDepth(image.depth());

    cv::Mat filter;
    cv::Mat kernel_x = (cv::Mat_<int>(5, 1) << 1, 1, 0, -1, -1);
    cv::Mat kernel_y = (cv::Mat_<int>(1, 5) << -1, -1, 0, 1, 1);
    cv::sepFilter2D(image, filter, new_depth, kernel_x, kernel_y);

    return filter;
}

}  // namespace nie
