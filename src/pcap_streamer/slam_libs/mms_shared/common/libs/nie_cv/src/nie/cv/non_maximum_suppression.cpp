/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "non_maximum_suppression.hpp"

#include <glog/logging.h>

/*
 * Implementation in this file is partly based on algorithm 4 from "Efficient
 * non-maximum suppression" by Neubeck and Van Gool, ICPR, August 2006.
 */

namespace detail {

// Convenience type for later update
using IndexT = int;

/*
 * Helper function that determines whether the supplied candidate extreme point
 * is indeed the only maximum in its neighborhood of size min_distance in
 * [pixel] in every direction, meaning that it returns false when there actually
 * is a better one in the neighborhood.
 */
bool IsExtremeWithinNeighborhood(
    cv::Mat const& image, cv::Point const& candidate, IndexT min_distance, bool is_minimum = false) {
    cv::Point window_end = {1, 1};
    cv::Point window_start = candidate - window_end * min_distance;   // top-left (inclusive)
    window_end = window_start + window_end * (2 * min_distance + 1);  // bottom-right (exclusive)

    cv::Rect current_window(window_start, window_end);
    current_window &= {0, 0, image.cols, image.rows};  // Intersection

    cv::Point point;
    cv::minMaxLoc(
        image(current_window), nullptr, nullptr, (is_minimum ? &point : nullptr), (not is_minimum ? &point : nullptr));
    point += window_start;

    return point == candidate;
}

}  // namespace detail

namespace nie {

void FindLocalExtremes(
    cv::Mat const& image,
    int min_distance,
    std::vector<cv::Point2f>* p_features,
    std::vector<bool>* p_feature_types,
    bool max_only) {
    // Convenience variables
    auto& features = *p_features;
    auto& feature_types = *p_feature_types;

    // Initialize a sufficiently large result vector
    std::size_t const estimated_increase =
        2 * (uint)(std::ceil((float)image.cols * image.rows / min_distance / min_distance));
    features.reserve(features.size() + estimated_increase);
    feature_types.reserve(feature_types.size() + estimated_increase);

    // Loop over minimal-distance-sized windows
    for (detail::IndexT c = min_distance; c < image.cols - min_distance; c += min_distance + 1) {
        for (detail::IndexT r = min_distance; r < image.rows - min_distance; r += min_distance + 1) {
            // Candidate extreme position
            cv::Point min_point;
            cv::Point max_point;

            // Loop over pixels of one window to find extreme value in window
            cv::Rect current_window(c, r, min_distance + 1, min_distance + 1);
            cv::minMaxLoc(
                image(current_window),
                /* min value*/ nullptr,
                /* max value*/ nullptr,
                (not max_only ? &min_point : nullptr),
                &max_point);

            // Check candidate minimum
            if (not max_only) {
                min_point += {c, r};
                if (detail::IsExtremeWithinNeighborhood(
                        image,
                        min_point,
                        min_distance, /*is_minimum*/
                        true)) {
                    features.emplace_back(min_point);
                    feature_types.emplace_back(true);
                }
            }
            // Check candidate maximum
            max_point += {c, r};
            if (detail::IsExtremeWithinNeighborhood(image, max_point, min_distance)) {
                features.emplace_back(max_point);
                feature_types.emplace_back(false);
            }
        }
    }
}

}  // namespace nie
