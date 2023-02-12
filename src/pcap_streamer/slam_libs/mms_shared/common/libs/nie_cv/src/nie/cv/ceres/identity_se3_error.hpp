/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LIDAR_CERES_IDENTITY_SE3_ERROR_HPP
#define NIE_LIDAR_CERES_IDENTITY_SE3_ERROR_HPP

#include <nie/core/geometry/isometry3.hpp>

namespace nie {

// TODO(jbr): Remove sqrt_information in the future. Use weighted cost function.

template <typename LeftDerived, typename LeftRotation, typename RightDerived, typename RightRotation>
inline void CalculateIdentityError(
    Isometry3Base<LeftDerived, LeftRotation> const& t_left,
    Isometry3Base<RightDerived, RightRotation> const& t_right,
    Eigen::Matrix<typename LeftDerived::NonConstScalar, 6, 6> const& sqrt_information,
    typename LeftDerived::NonConstScalarPointer p_residuals) {
    using T = typename LeftDerived::NonConstScalar;

    // The translation part is directly mapped to the residuals pointer.
    Eigen::Quaternion<T> delta_q;
    nie::Isometry3Map<Eigen::Quaternion<T>> t_delta(p_residuals, delta_q.coeffs().data());

    // Compute the residuals between the two transforms t_left - t_right.
    t_delta = t_left.Delta(t_right);

    // Update orientation residual (position one was mapped via t_delta)
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(p_residuals);
    // No need to set the delta position since it was mapped via t_delta
    residuals.template block<3, 1>(3, 0) = T(2.0) * t_delta.rotation().vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information);
}

}  // namespace nie

#endif  // NIE_LIDAR_CERES_IDENTITY_SE3_ERROR_HPP
