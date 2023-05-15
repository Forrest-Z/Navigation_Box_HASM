/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <functional>
#include <vector>

#include <opencv2/core.hpp>

namespace nie {

// The in argument is not by reference to prevent that in and out are the same object. This would cause an error.
template <typename T>
void TransformPerspective(cv::Matx33d const& matrix, cv::Point_<T> const in, cv::Point_<T>* out) {
    double w = matrix(2, 0) * in.x + matrix(2, 1) * in.y + matrix(2, 2);

    out->x = static_cast<T>((matrix(0, 0) * in.x + matrix(0, 1) * in.y + matrix(0, 2)) / w);
    out->y = static_cast<T>((matrix(1, 0) * in.x + matrix(1, 1) * in.y + matrix(1, 2)) / w);
}

template <typename T>
void TransformPerspective(
    cv::Matx33d const& matrix, std::vector<cv::Point_<T>> const& in, std::vector<cv::Point_<T>>* out) {
    // Should be the same as the for loop for the single point version, opencv doesn't have a single point version
    // however.
    cv::perspectiveTransform(in, *out, matrix);
}

cv::Mat GetLookUpTable(
    std::function<bool(cv::Point2f const& p_u, cv::Point2f* p_d)> const& transformer, cv::Size const& size);

}  // namespace nie
