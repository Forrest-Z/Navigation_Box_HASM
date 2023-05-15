/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_NON_MAXIMUM_SUPPRESSION_HPP
#define NIE_CV_NON_MAXIMUM_SUPPRESSION_HPP

#include <vector>

#include <opencv2/opencv.hpp>

namespace nie {

/*
 * Find positions of extreme values in a matrix at least separated by the
 * parameter min_distance [pixel]. By default both minimum and maximum value are
 * returned. When parameter max_only is set to true, then only maximum values
 * are considered.
 *
 * Notes
 *  - The extremes returned are only min_distance separated when of the
 *    same type, begin same type of extreme value (min/max) in the matrix.
 *  - When two extremes have the same value, but too close together, then it
 *    might be possible that both are selected. (This is due to the windowing.)
 */
void FindLocalExtremes(
    cv::Mat const& images,
    int min_distance,
    std::vector<cv::Point2f>* features,
    std::vector<bool>* feature_types,
    bool max_only = false);

}  // namespace nie

#endif  // NIE_CV_NON_MAXIMUM_SUPPRESSION_HPP
