/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_ODOMETRY_H
#define LOAM_ODOMETRY_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "message_header.hpp"

namespace loam {

struct PoseWithCovariance {
    struct Pose {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    };

    Pose pose;
    Eigen::Matrix<double, 36, 1> covariance = Eigen::Matrix<double, 36, 1>::Zero();
};

struct TwistWithCovariance {
    struct Twist {
        Eigen::Vector3d linear = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    };

    Twist twist;
    Eigen::Matrix<double, 36, 1> covariance = Eigen::Matrix<double, 36, 1>::Zero();
};

struct Odometry {
    MessageHeader header;
    // This is the frame_id of the children
    std::string child_frame;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

}  // end namespace loam

#endif  // LOAM_ODOMETRY_H
