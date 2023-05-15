/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <Eigen/Eigen>

namespace nie {

struct RegisteredPair {
    nie::io::PoseId id_a;
    nie::io::PoseId id_b;

    nie::Timestamp_ns timestamp_a;
    nie::Timestamp_ns timestamp_b;

    nie::Isometry3qd bbox_pose_abs_a;
    nie::Isometry3qd bbox_pose_abs_b;

    // Transformations converting points from b to a (or converting coordinate frame a to b)
    nie::Isometry3qd T_ab_icp;  // Result of ICP, refinement of initial guess

    std::string filename_a;
    std::string filename_b;
};

}  // namespace nie
