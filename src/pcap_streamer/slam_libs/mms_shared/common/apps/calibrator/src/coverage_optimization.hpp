/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef COVERAGE_OPTIMIZATION_HPP
#define COVERAGE_OPTIMIZATION_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

void OptimizeSpatialCoverage(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::string>* image_ids,
    std::vector<std::vector<cv::Point2f>>* image_points);

void OptimizeSpatialCoverage(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::size_t pair_count,
    std::vector<std::string>* image_ids_left,
    std::vector<std::string>* image_ids_right,
    std::vector<std::vector<cv::Point2f>>* image_points_left,
    std::vector<std::vector<cv::Point2f>>* image_points_right,
    std::size_t* optimized_pair_count);

#endif  // COVERAGE_OPTIMIZATION_HPP