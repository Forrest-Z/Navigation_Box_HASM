/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <eigen3/Eigen/Core>

namespace nie {

/// Fixes euler singularity if occuring.
///
/// \param euler_angles [in/out] vector containing euler angles
void CorrectEulerSingularity(Eigen::Vector3d* euler_angles);

}  // namespace nie
