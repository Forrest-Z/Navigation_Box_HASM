/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_CALIB3D_NORMALIZATION_HPP
#define NIE_CV_CALIB3D_NORMALIZATION_HPP

#include <vector>

#include <opencv2/core.hpp>

namespace nie {

template <typename T>
cv::Matx33d ComputeNormalizationMatrix(std::vector<cv::Point_<T>> const& points) {
    constexpr double sqrt_2 = std::sqrt(2.0);

    cv::Point2d avg;

    for (auto&& point : points) {
        avg += point.operator cv::Point2d();
    }

    avg *= 1.0 / static_cast<double>(points.size());

    double stddev = 0.0;

    for (auto&& point : points) {
        stddev += cv::norm(point.operator cv::Point2d() - avg);
    }

    // Average distance from the center will be sqrt (2) Vector (1, 1)
    // Choice of Hartley and Zisserman. Just a choice.
    stddev = stddev / (sqrt_2 * static_cast<double>(points.size()));

    return cv::Matx33d{1.0 / stddev,
                       0.0,
                       -avg.x / stddev,  // p.x * (1.0 / stddev) + p.y * 0.0 + 1.0 * (-avg.x / stddev)
                       0.0,
                       1.0 / stddev,
                       -avg.y / stddev,
                       0.0,
                       0.0,
                       1.0};
}

inline cv::Matx33d ComputeNormalizationMatrix(cv::Scalar const& avg, cv::Scalar const& stddev) {
    constexpr double sqrt_2 = std::sqrt(2.0);

    // Average distance from the center will be sqrt (2) Vector (1, 1)
    // Choice of Hartley and Zisserman. Just a choice.
    double s = std::sqrt(stddev[0] * stddev[0] + stddev[1] * stddev[1]) / sqrt_2;

    return cv::Matx33d{1.0 / s, 0.0, -avg[0] / s, 0.0, 1.0 / s, -avg[1] / s, 0.0, 0.0, 1.0};
}

}  // namespace nie

#endif  // NIE_CV_CALIB3D_NORMALIZATION_HPP
