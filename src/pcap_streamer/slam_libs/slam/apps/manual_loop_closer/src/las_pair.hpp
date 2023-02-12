/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/time.hpp>

namespace nie {

struct LasPair {
    nie::Isometry3qd bbox_pose_a;
    nie::Isometry3qd bbox_pose_b;

    nie::Timestamp_ns timestamp_a;
    nie::Timestamp_ns timestamp_b;

    std::string filename_a;
    std::string filename_b;
};

}  // namespace nie
