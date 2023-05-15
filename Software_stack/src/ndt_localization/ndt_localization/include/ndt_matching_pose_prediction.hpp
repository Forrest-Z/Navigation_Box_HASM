/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#pragma once

#include "ndt_matching_types.hpp"

namespace nie {

namespace ndt {

// TODO(tvds): Validate equations used for the predict_pose_* functions

/** Predicts pose assuming a linear relation using estimated velocity
 *
 * @param pose: Reference pose
 * @param velocity: Linear and angular velocities to use for prediction
 * @param dt: Time since last pose in seconds
 *
 * @return Estimated pose
 */
Pose PredictPoseLinear(Pose const& pose, RateOfChange const& velocity, double dt);

/** Predicts pose assuming a quadratic relation using estimated velocity and acceleration
 *
 * @param pose: Reference pose
 * @param velocity: Linear and angular velocities to use for prediction
 * @param acceleration: Linear and angular accelerations to use for prediction
 * @param dt: Time since last pose in seconds
 *
 * @return Estimated pose
 */
Pose PredictPoseQuadratic(Pose const& pose, RateOfChange const& velocity, RateOfChange const& acceleration, double dt);

}  // namespace ndt

}  // namespace nie
