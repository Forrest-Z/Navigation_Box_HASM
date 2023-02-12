/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_VO_CREATE_VISUAL_ODOMETRY_HPP
#define NIE_VO_CREATE_VISUAL_ODOMETRY_HPP

#include <nie/cv/vo/visual_odometry_mono.hpp>

namespace nie {

VisualOdometryMonoPtr CreateVisualOdometryMono(
    int image_width, int image_height, VisualOdometryMono::Parameters parameters, bool draw = false);

}  // namespace nie

#endif  // NIE_VO_CREATE_VISUAL_ODOMETRY_HPP
