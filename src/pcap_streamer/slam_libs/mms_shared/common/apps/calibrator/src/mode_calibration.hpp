/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef MODE_CALIBRATION_HPP
#define MODE_CALIBRATION_HPP

#include <string>

#include <opencv2/core.hpp>

void RunModeCalibration(
    std::string const& camera_id,
    std::string const& in_path_images,
    std::string const& out_path_intrinsics,
    std::string const& out_path_extended_calibration_data,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    float const& square_size,
    bool write_corner_images);

void RunModeCalibration(
    std::string const& camera_id_left,
    std::string const& camera_id_right,
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& out_path_intrinsics,
    std::string const& out_path_extended_calibration_data,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    float const& square_size,
    bool write_corner_images);

#endif  // MODE_CALIBRATION_HPP
