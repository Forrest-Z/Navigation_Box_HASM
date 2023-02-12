/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef MODE_VALIDATION_HPP
#define MODE_VALIDATION_HPP

#include <string>

#include <opencv2/core.hpp>

void RunModeValidation(
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& in_path_intrinsics,
    std::string const& in_path_extended,
    std::string const& out_path_validated,
    bool write_corner_images,
    bool stereo);

#endif  // MODE_VALIDATION_HPP
