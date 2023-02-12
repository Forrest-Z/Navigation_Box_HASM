/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef MODE_RECORDING_HPP
#define MODE_RECORDING_HPP

#include <string>

#include <nie/hardware/camera/basler_camera.hpp>
#include <opencv2/core.hpp>
#include "checkerboard_overlap_filter.hpp"

namespace nie {

namespace detail {

const int kPreviewTargetWidth = 320;
const int kPreviewCaptureSpeedMs = 50;
const float kOnlineOverlapThreshold = 0.80f;

}  // namespace detail

}  // namespace nie

struct StereoFolderStructure {
    std::string root_folder;
    std::string path_relative_left;
    std::string path_relative_right;
};

void WriteImage(std::unique_ptr<std::pair<std::string, cv::Mat>> const& work);

cv::Size GetPreviewSizeFromTargetWidth(nie::BaslerCamera const& camera, int preview_target_width);

std::size_t RunModeRecording(
        std::string const& out_path_images, nie::BaslerCamera* camera, std::unique_ptr<nie::BaseFilter> const& filter);

std::size_t RunModeRecording(
        StereoFolderStructure const& out_folder_struct,
        cv::Size const& pattern_size,
        int const gpio_port,
        nie::BaslerCamera* camera_left,
        nie::BaslerCamera* camera_right);

#endif  // MODE_RECORDING_HPP