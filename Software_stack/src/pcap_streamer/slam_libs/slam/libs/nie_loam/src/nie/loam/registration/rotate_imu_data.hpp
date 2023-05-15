/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "nie/loam/ros_interop/imu_message.hpp"
#include "nie/loam/ros_interop/transform_stamped.hpp"

namespace loam {

/**
 * Transforms a covariance array from one frame to another
 */
inline void transformCovariance(
    Eigen::Matrix<double, 9, 1> const& in, Eigen::Matrix<double, 9, 1>* out, Eigen::Quaternion<double> r) {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_out(out->data());
    cov_out = r * cov_in * r.inverse();
}

/**
 * Transforms ImuMessage data from one frame to another
 */
inline void transformIMU(ImuMessage const& imu_in, ImuMessage& imu_out, TransformStamped const& t_in) {
    imu_out.header = t_in.header;

    // Discard translation, only use orientation for IMU transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w(),
        t_in.transform.rotation.x(),
        t_in.transform.rotation.y(),
        t_in.transform.rotation.z());
    Eigen::Transform<double, 3, Eigen::Affine> t(r);

    Eigen::Vector3d vel =
        t * Eigen::Vector3d(imu_in.angular_velocity.x(), imu_in.angular_velocity.y(), imu_in.angular_velocity.z());

    imu_out.angular_velocity.x() = vel.x();
    imu_out.angular_velocity.y() = vel.y();
    imu_out.angular_velocity.z() = vel.z();

    transformCovariance(imu_in.angular_velocity_cov, &imu_out.angular_velocity_cov, r);

    Eigen::Vector3d accel =
        t *
        Eigen::Vector3d(imu_in.linear_acceleration.x(), imu_in.linear_acceleration.y(), imu_in.linear_acceleration.z());

    imu_out.linear_acceleration.x() = accel.x();
    imu_out.linear_acceleration.y() = accel.y();
    imu_out.linear_acceleration.z() = accel.z();

    transformCovariance(imu_in.linear_acceleration_cov, &imu_out.linear_acceleration_cov, r);

    Eigen::Quaternion<double> orientation =
        r *
        Eigen::Quaternion<double>(
            imu_in.orientation.w(), imu_in.orientation.x(), imu_in.orientation.y(), imu_in.orientation.z()) *
        r.inverse();

    imu_out.orientation.w() = orientation.w();
    imu_out.orientation.x() = orientation.x();
    imu_out.orientation.y() = orientation.y();
    imu_out.orientation.z() = orientation.z();

    transformCovariance(imu_in.orientation_cov, &imu_out.orientation_cov, r);
}

}  // namespace loam
