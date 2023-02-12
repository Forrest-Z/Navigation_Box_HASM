/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_IMU_MESSAGE_H
#define LOAM_IMU_MESSAGE_H

#include <cmath>

#include <Eigen/Core>

#include "message_header.hpp"

namespace loam {

struct ImuMessage {
    // From the ROS Imu Message documentation:
    // Cov Unknown = zeroes
    // Measurement Unknown = element 0 of cov == -1
    // Our real IMU data for HAD does not output orientations
    ImuMessage()
        : header(),
          orientation(Eigen::Quaterniond::Identity()),
          orientation_cov(Eigen::Matrix<double, 9, 1>::Zero()),
          angular_velocity(Eigen::Vector3d::Zero()),
          angular_velocity_cov(Eigen::Matrix<double, 9, 1>::Zero()),
          linear_acceleration(Eigen::Vector3d::Zero()),
          linear_acceleration_cov(Eigen::Matrix<double, 9, 1>::Zero()) {
        orientation_cov[0] = -1;
    }

    MessageHeader header;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double, 9, 1> orientation_cov;
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix<double, 9, 1> angular_velocity_cov;
    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix<double, 9, 1> linear_acceleration_cov;
};

}  // end namespace loam

#endif  // LOAM_IMU_MESSAGE_H
