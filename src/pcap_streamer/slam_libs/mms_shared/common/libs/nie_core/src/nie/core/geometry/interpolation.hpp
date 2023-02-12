/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <glog/logging.h>
#include <Eigen/Eigenvalues>

#include "covariance.hpp"
#include "isometry3.hpp"

namespace nie {

/// Compute linear interpolation between a pair of 3-dimensional translations.
///
/// \param t1
/// \param t2
/// \param ratio  scalar \in [0, 1]
/// \return  Interpolated translation.
template <
        typename Derived1,
        typename Derived2,
        typename Scalar = typename Eigen::MatrixBase<Derived1>::Scalar,
        int Rows = Derived1::RowsAtCompileTime,
        int Cols = Derived1::ColsAtCompileTime>
Eigen::Matrix<Scalar, Rows, Cols> Interpolate(
        Eigen::MatrixBase<Derived1> const& t1, Eigen::MatrixBase<Derived2> const& t2, Scalar const& ratio) {
    static_assert(
            Derived1::RowsAtCompileTime == Derived2::RowsAtCompileTime, "Interpolation needs equal size matrices.");
    static_assert(
            Derived1::ColsAtCompileTime == Derived2::ColsAtCompileTime, "Interpolation needs equal size matrices.");

    DCHECK(ratio >= 0.0) << "Interpolation ratio must be greater than or equal to 0.0.";
    DCHECK(ratio <= 1.0) << "Interpolation ratio must be lesser than or equal to 1.0.";
    return t1 + (t2 - t1) * ratio;
}

/// Check if Quaternion is normalized.
template <typename Derived>
constexpr bool IsQuaternionNormalized(Eigen::QuaternionBase<Derived> const& q1) {
    using std::abs;  // To find ceres JET type overloaded abs function too
    using Scalar = typename Eigen::QuaternionBase<Derived>::Scalar;
    Scalar const kTolerance = static_cast<Scalar>(std::numeric_limits<double>::epsilon() * 10.0);
    return abs(q1.norm() - Scalar(1.0)) < kTolerance;
}

/// Compute spherical linear interpolation between a pair of normalized quaternions. Represents constant-speed motion.
///
/// @precondition both @param{q1} and @param{q2} must be normalized quaternions.
///
/// \param q1
/// \param q2
/// \param ratio  scalar \in [0, 1]
/// \return  Interpolated normalized quaternion.
template <typename Derived1, typename Derived2>
Eigen::Quaternion<typename Eigen::QuaternionBase<Derived1>::Scalar> Interpolate(
        Eigen::QuaternionBase<Derived1> const& q1,
        Eigen::QuaternionBase<Derived2> const& q2,
        typename Eigen::QuaternionBase<Derived1>::Scalar const& ratio) {
    DCHECK(IsQuaternionNormalized(q1)) << "Interpolation needs normalized quaternions.";
    DCHECK(IsQuaternionNormalized(q2)) << "Interpolation needs normalized quaternions.";

    DCHECK(ratio >= 0.0) << "Interpolation ratio must be greater than or equal to 0.0.";
    DCHECK(ratio <= 1.0) << "Interpolation ratio must be lesser than or equal to 1.0.";

    return q1.slerp(ratio, q2);
}

/// Interpolate translation and rotation with factor @param{ratio}. A @param{ratio} value of zero returns @param{isom1}
///  and a @param{ratio} of 1.0 returns @param{isom2}.
///
/// @precondition both @param{isom1} and @param{isom2} must contain normalized quaternions.
///
/// \param isom1
/// \param isom2
/// \param ratio  scalar \in [0, 1]
/// \return  Interpolated isometry.
template <typename Derived1, typename Rotation1, typename Derived2, typename Rotation2>
Isometry3<Rotation1> Interpolate(
        Isometry3Base<Derived1, Rotation1> const& isom1,
        Isometry3Base<Derived2, Rotation2> const& isom2,
        typename Isometry3Base<Derived1, Rotation1>::Scalar const& ratio) {
    return {Interpolate(isom1.translation(), isom2.translation(), ratio),
            Interpolate(isom1.rotation(), isom2.rotation(), ratio)};
}

/// Compute interpolation between a pair of covariance matrices.
///
/// \param m1
/// \param m2
/// \param ratio  scalar \in [0, 1]
/// \return  Interpolated covariance matrix
template <
        typename Derived1,
        typename Derived2,
        typename Scalar = typename Eigen::MatrixBase<Derived1>::Scalar,
        int N = Derived1::RowsAtCompileTime,
        typename ResultVector = typename Eigen::Matrix<Scalar, N, 1>,
        typename ResultMatrix = typename Eigen::Matrix<Scalar, N, N>>
ResultMatrix CovInterpolate(
        Eigen::MatrixBase<Derived1> const& m1, Eigen::MatrixBase<Derived2> const& m2, Scalar const& ratio) {
    static_assert(N == Derived1::ColsAtCompileTime, "CovInterpolation needs square matrices.");
    static_assert(N == Derived2::RowsAtCompileTime, "CovInterpolation needs equal size matrices.");
    static_assert(N == Derived2::ColsAtCompileTime, "CovInterpolation needs equal size matrices.");

    DCHECK(m1.isApprox(m1.adjoint())) << "CovInterpolation first matrix is not selfadjoint.";
    DCHECK(m2.isApprox(m2.adjoint())) << "CovInterpolation second matrix is not selfadjoint.";
    DCHECK(ratio >= 0.0) << "CovInterpolation ratio must be greater than or equal to 0.0.";
    DCHECK(ratio <= 1.0) << "CovInterpolation ratio must be lesser than or equal to 1.0.";

    // As the matrix is selfadjoint, the eigen values (and vectors) are real (not complex) valued. The eigen values are
    // sorted for the SelfAdjointEigenSolver, so interpolation can be done (almost) immediately. Effectively the biggest
    // variance is interpolated towards the other biggest, 2nd biggest to the other 2nd biggest and so on.

    Eigen::SelfAdjointEigenSolver<Derived1> es1{m1};
    Eigen::SelfAdjointEigenSolver<Derived2> es2{m2};

    // When the vectors of similar eigenvalues are in roughly opposite directions, then the interpolated vectors will be
    // wrong and hence the resulting matrix. Therefore the vectors of one matrix can be flipped so that every pair is in
    // roughly the same direction.
    ResultMatrix const& ev1 = es1.eigenvectors();
    ResultMatrix ev2 = es2.eigenvectors();
    for (std::size_t i = 0; i < N; ++i) {
        if (ev1.col(i).dot(ev2.col(i)) < 0.) {
            ev2.col(i) *= -1.;
        }
    }

    ResultVector const lm = Interpolate(es1.eigenvalues(), es2.eigenvalues(), ratio);
    ResultMatrix const vm = Interpolate(ev1, ev2, ratio);
    return vm * lm.asDiagonal() * vm.transpose();
}

/// Compute interpolation between a pair of information matrices. Uses CovInterpolate function
///
/// \param m1
/// \param m2
/// \param ratio  scalar \in [0, 1]
/// \return  Interpolated information matrix
template <typename Derived1, typename Derived2, typename Scalar = typename Eigen::MatrixBase<Derived1>::Scalar>
auto InfInterpolate(
        Eigen::MatrixBase<Derived1> const& inf_1, Eigen::MatrixBase<Derived2> const& inf_2, Scalar const& ratio) {
    Derived1 cov_1;
    Derived2 cov_2;
    InformationToCovariance(inf_1, &cov_1);
    InformationToCovariance(inf_2, &cov_2);
    Derived1 const cov_m = CovInterpolate(cov_1, cov_2, ratio);

    Derived1 inf_m;
    CovarianceToInformation(cov_m, &inf_m);
    return inf_m;
}

}  // namespace nie
