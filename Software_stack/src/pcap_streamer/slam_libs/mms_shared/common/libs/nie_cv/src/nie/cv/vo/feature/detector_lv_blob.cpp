/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "detector_lv_blob.hpp"

#include <nie/core/scoped_timer.hpp>

#include <opencv2/opencv.hpp>

namespace nie {

void DetectorLvBlob::Detect(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) {
    ScopedTimer timer("DetectorLvBlob::Detect");

    cv::Mat const filter = Filter(image);
    FindFeatures(filter, p_features, p_feature_types);

    Callback<Handle::kFilter>(filter, *p_features);
}

cv::Mat DetectorLvBlob::Filter(cv::Mat const& image) {
    ScopedTimer timer("DetectorLvBlob::Filter");
    /*
     * The full filter looks like:
     *   |-1 -1 -1 -1 -1|
     *   |-1  1  1  1 -1|
     *   |-1  1  8  1 -1|
     *   |-1  1  1  1 -1|
     *   |-1 -1 -1 -1 -1|
     * which can be treated as being a combination of multiple blur filters
     * like:
     *        |1 1 1 1 1|
     *        |1 1 1 1 1|       |1 1 1|
     *   -1 * |1 1 1 1 1| + 2 * |1 1 1| + 7 * |1|
     *        |1 1 1 1 1|       |1 1 1|
     *        |1 1 1 1 1|
     *
     * This method is slightly faster than using the full kernel in combination
     * with the filter2D function.
     */

    // Make sure filter image can fit the calculated values
    int const new_depth = IncreaseDepth(image.depth());

    // +7 central pixel
    cv::Mat filter;
    image.convertTo(filter, new_depth, /* scale/factor */ 7);

    cv::Point const anchor{-1, -1};  // default anchor in middle of kernel
    bool const normalize = false;    // no normalization on individual filters should be applied

    // +2 3x3 window
    cv::Mat tmp;
    cv::boxFilter(image, tmp, new_depth, cv::Size(3, 3), anchor, normalize);
    filter += 2 * tmp;

    // -1 5x5 window
    cv::boxFilter(image, tmp, new_depth, cv::Size(5, 5), anchor, normalize);
    filter -= tmp;

    return filter;
}

}  // namespace nie
