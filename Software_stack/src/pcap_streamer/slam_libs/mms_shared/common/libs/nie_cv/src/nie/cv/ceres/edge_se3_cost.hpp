/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LIDAR_CERES_EDGE_SE3_COST_HPP
#define NIE_LIDAR_CERES_EDGE_SE3_COST_HPP

#include <ceres/autodiff_cost_function.h>
#include <nie/core/geometry/isometry3.hpp>

#include "identity_se3_error.hpp"

namespace nie {

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1} * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
//
// where I is the information matrix which is the inverse of the covariance.
class EdgeSe3Cost {
public:
    EdgeSe3Cost(Isometry3qd t_ab_measured, Eigen::Matrix<double, 6, 6> sqrt_information)
        : t_ab_measured_(std::move(t_ab_measured)), sqrt_information_(std::move(sqrt_information)) {}

    template <typename T>
    bool operator()(
            const T* const p_t_a, const T* const p_r_a, const T* const p_t_b, const T* const p_r_b, T* p_residuals)
            const {
        nie::Isometry3Map<const Eigen::Quaternion<T>> t_a(p_t_a, p_r_a);
        nie::Isometry3Map<const Eigen::Quaternion<T>> t_b(p_t_b, p_r_b);

        // Compute the relative transformation between the two frames.
        nie::Isometry3q<T> t_ab_estimated = t_a.TransformInverseLeft(t_b);

        CalculateIdentityError(
                t_ab_estimated, t_ab_measured_.template cast<T>(), sqrt_information_.template cast<T>(), p_residuals);

        return true;
    }

    static ceres::CostFunction* Create(Isometry3qd const& t_ab_measured, Eigen::Matrix<double, 6, 6> sqrt_information) {
        return new ceres::AutoDiffCostFunction<EdgeSe3Cost, 6, 3, 4, 3, 4>(
                new EdgeSe3Cost(t_ab_measured, std::move(sqrt_information)));
    }

private:
    // The measurement for the position of B relative to A in the A frame.
    Isometry3qd const t_ab_measured_;
    // The square root of the measurement information matrix.
    Eigen::Matrix<double, 6, 6> const sqrt_information_;
};

}  // namespace nie

#endif  // NIE_LIDAR_CERES_EDGE_SE3_COST_HPP