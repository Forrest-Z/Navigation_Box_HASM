/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "correct_euler_singularity.hpp"

#include <nie/core/constants.hpp>

#include <glog/logging.h>

namespace nie {

void CorrectEulerSingularity(Eigen::Vector3d* euler_angles) {
    // Check for euler singularity
    // e.g. all angles +/-180.0 degrees == all angles 0.0 degrees
    // We can fix this by rotating all angles +/- 180 degrees.
    // The order or sign does not matter, as long as its done for all angles
    // We do this once all angles are bigger than 90 degrees, as then we can bring them all closer to 0 degrees by
    // applying this change.
    double constexpr angle_threshold = M_PI / 2.0;
    if ((euler_angles->array().abs() > angle_threshold).all()) {
        (*euler_angles)[0] += (*euler_angles)[0] > 0.0 ? -nie::kPi<> : nie::kPi<>;
        (*euler_angles)[1] += (*euler_angles)[1] > 0.0 ? -nie::kPi<> : nie::kPi<>;
        (*euler_angles)[2] += (*euler_angles)[2] > 0.0 ? -nie::kPi<> : nie::kPi<>;
    }
}

}  // namespace nie