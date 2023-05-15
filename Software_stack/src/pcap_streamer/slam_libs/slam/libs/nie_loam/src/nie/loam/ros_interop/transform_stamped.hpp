/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_TRANSFORM_STAMPED_H
#define LOAM_TRANSFORM_STAMPED_H

#include <cmath>

#include <Eigen/Core>

#include "message_header.hpp"

namespace loam {

struct Transform {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

struct TransformStamped {
    MessageHeader header;
    // This is the frame_id of the child transform, not necessarily the same as header.frame_id
    std::string child_fame_id;
    Transform transform;
};

}  // end namespace loam

#endif  // LOAM_TRANSFORM_STAMPED_H
