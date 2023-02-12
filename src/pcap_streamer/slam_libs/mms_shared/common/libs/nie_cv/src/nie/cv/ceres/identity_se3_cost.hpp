/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LIDAR_CERES_IDENTITY_SE3_COST_HPP
#define NIE_LIDAR_CERES_IDENTITY_SE3_COST_HPP

#include <ceres/autodiff_cost_function.h>
#include <nie/core/geometry/isometry3.hpp>

#include "identity_se3_error.hpp"

namespace nie {

// Computes the identity error term for one pose. Let the hat variables be
// the measurement. We can compute an error metric between the current
// estimate of the pose and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].
//
// Now we can compute an error metric between the estimated and measurement
// transformation. For the orientation error, we will use the standard
// multiplicative error resulting in:
//
//   error = [ p - \hat{p}                 ]
//           [ 2.0 * Vec(q * \hat{q}^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
//
// where I is the information matrix which is the inverse of the covariance.
class IdentitySe3Cost {
public:
    IdentitySe3Cost(Isometry3qd t_measured, Eigen::Matrix<double, 6, 6> sqrt_information)
        : t_measured_(std::move(t_measured)), sqrt_information_(std::move(sqrt_information)) {}

    template <typename T>
    bool operator()(const T* const p_t, const T* const p_r, T* p_residuals) const {
        nie::Isometry3Map<const Eigen::Quaternion<T>> t_estimated(p_t, p_r);

        CalculateIdentityError(
            t_estimated, t_measured_.template cast<T>(), sqrt_information_.template cast<T>(), p_residuals);

        return true;
    }

    static ceres::CostFunction* Create(const Isometry3qd& t_measured, Eigen::Matrix<double, 6, 6> sqrt_information) {
        return new ceres::AutoDiffCostFunction<IdentitySe3Cost, 6, 3, 4>(
            new IdentitySe3Cost(t_measured, std::move(sqrt_information)));
    }

private:
    // The measurement for the position.
    Isometry3qd const t_measured_;
    // The square root of the measurement information matrix.
    Eigen::Matrix<double, 6, 6> const sqrt_information_;
};

}  // namespace nie

#endif  // NIE_LIDAR_CERES_IDENTITY_SE3_COST_HPP
