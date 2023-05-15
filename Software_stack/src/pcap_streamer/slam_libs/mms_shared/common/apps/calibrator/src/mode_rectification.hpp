/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef MODE_RECTIFICATION_HPP
#define MODE_RECTIFICATION_HPP

#include <string>

void RunModeRectification(
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& in_path_intrinsics,
    std::string const& out_path_images,
    std::string const& out_path_intrinsics,
    bool stereo);

#endif  // MODE_RECTIFICATION_HPP
