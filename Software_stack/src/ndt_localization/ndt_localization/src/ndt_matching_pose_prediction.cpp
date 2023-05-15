/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#include "ndt_matching_pose_prediction.hpp"

namespace nie {

namespace ndt {

Pose PredictPoseLinear(Pose const& pose, RateOfChange const& velocity, double dt) {
    Pose offset{};
    offset.x = velocity.linear_x * dt;
    offset.y = velocity.linear_y * dt;
    offset.z = velocity.linear_z * dt;
    // We are ignoring the roll and pitch.
    // These will likely be too noisy.
    offset.roll = 0;
    offset.pitch = 0;
    offset.yaw = velocity.angular_z * dt;
    return pose + offset;
}

Pose PredictPoseQuadratic(Pose const& pose, RateOfChange const& velocity, RateOfChange const& acceleration, double dt) {
    Pose offset{};
    offset.x = (velocity.linear_x + acceleration.linear_x * dt) * dt;
    offset.y = (velocity.linear_y + acceleration.linear_y * dt) * dt;
    offset.z = velocity.linear_z * dt;
    // We are ignoring the roll and pitch.
    // These will likely be too noisy.
    offset.roll = 0;
    offset.pitch = 0;
    offset.yaw = velocity.angular_z * dt;
    return pose + offset;
}

}  // namespace ndt

}  // namespace nie
