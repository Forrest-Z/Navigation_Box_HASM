/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Core>

namespace nie {

namespace detail {

template <typename T, int Rows, int Cols>
struct IdentityType {
    using Type = Eigen::Matrix<typename std::remove_const<T>::type, Rows, Cols>;

    template <typename Derived>
    inline static Type Identity(Eigen::MatrixBase<Derived> const&) {
        return Type::Identity();
    }
};

template <typename T>
struct IdentityType<T, Eigen::Dynamic, Eigen::Dynamic> {
    using Type = Eigen::Matrix<typename std::remove_const<T>::type, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename Derived>
    inline static Type Identity(Eigen::MatrixBase<Derived> const& matrix) {
        return Type::Identity(matrix.rows(), matrix.cols());
    }
};

template <
    typename Derived,
    typename T = IdentityType<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>>
typename T::Type Identity(Eigen::MatrixBase<Derived> const& matrix) {
    return T::Identity(matrix);
}

// Inverse of a symmetric, positive definite matrix
template <typename LeftDerived, typename RightDerived>
void SymPosDefMatrixInverse(Eigen::MatrixBase<LeftDerived> const& a, Eigen::MatrixBase<RightDerived>* b) {
    *b = a.llt().solve(detail::Identity(a));
}

}  // namespace detail

// Inverse of the covariance
template <typename LeftDerived, typename RightDerived>
void CovarianceToInformation(Eigen::MatrixBase<LeftDerived> const& cov, Eigen::MatrixBase<RightDerived>* inf) {
    return detail::SymPosDefMatrixInverse(cov, inf);
}
// Inverse of the information
template <typename LeftDerived, typename RightDerived>
void InformationToCovariance(Eigen::MatrixBase<LeftDerived> const& inf, Eigen::MatrixBase<RightDerived>* cov) {
    return detail::SymPosDefMatrixInverse(inf, cov);
}

// Sqrt of the information matrix
template <typename LeftDerived, typename RightDerived>
void InformationToWeights(Eigen::MatrixBase<LeftDerived> const& information, Eigen::MatrixBase<RightDerived>* weights) {
    *weights = information.llt().matrixL();
}

// Inverse sqrt of the covariance
template <typename LeftDerived, typename RightDerived>
void CovarianceToWeights(Eigen::MatrixBase<LeftDerived> const& covariance, Eigen::MatrixBase<RightDerived>* weights) {
    auto svd = covariance.bdcSvd(Eigen::DecompositionOptions::ComputeFullU);
    auto inv_sqrt_diagonal = svd.singularValues().array().inverse().sqrt().matrix().asDiagonal();
    *weights = svd.matrixU() * inv_sqrt_diagonal * svd.matrixU().transpose();
}

}  // namespace nie
