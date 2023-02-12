/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef VISUAL_ODOMETRY_MONO_DEMO_HPP
#define VISUAL_ODOMETRY_MONO_DEMO_HPP

#include <nie/cv/vo/visual_odometry_mono.hpp>

void AddDrawingCallbacks(
    std::vector<std::string> const& image_files, float scale, int image_type, nie::VisualOdometryMonoPtr* p_vo);

#endif  // VISUAL_ODOMETRY_MONO_DEMO_HPP
