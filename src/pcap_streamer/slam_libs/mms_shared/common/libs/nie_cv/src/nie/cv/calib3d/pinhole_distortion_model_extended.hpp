/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_PINHOLE_MODEL_EXTENDED_HPP
#define NIE_CV_CALIB3D_PINHOLE_MODEL_EXTENDED_HPP

#include "nie/cv/ceres/helper_ceres.hpp"
#include "nie/cv/ceres/identity_cost_function.hpp"
#include "nie/cv/ceres/pinhole_distort_cost.hpp"
#include "nie/cv/ceres/pinhole_undistort_cost.hpp"
#include "nie/cv/ceres/weighted_cost_function.hpp"
#include "pinhole_distortion_model.hpp"

namespace nie {

// Returns 2 output stddevs in a Point_ of type T
template <typename T>
Eigen::Matrix<T, 2, 1> GetStddevs2(ceres::Covariance const& covariance, double* const data) {
    std::vector<double> covariance_matrix(4);
    covariance.GetCovarianceBlock(data, data, covariance_matrix.data());

    return Eigen::Matrix<T, 2, 1>(
        static_cast<T>(std::sqrt(covariance_matrix[0])), static_cast<T>(std::sqrt(covariance_matrix[3])));
}

// TODO(jbr): Refactor using covariance.

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeDistortionModelExtended final : public PinholeDistortionModel<FL, SK, RA, TA, TP> {
private:
    using Base = PinholeDistortionModel<FL, SK, RA, TA, TP>;

public:
    PinholeDistortionModelExtended() {
        Base::intrinsics_.resize(Base::kParametersIntrinsics, 0);
        stddevs_.resize(Base::kParametersIntrinsics, 1);
    }
    explicit PinholeDistortionModelExtended(const Eigen::Matrix3d& K)
        : Base(K), stddevs_(Base::kParametersIntrinsics, 1) {}
    PinholeDistortionModelExtended(std::vector<double> intrinsics, std::vector<double> stddevs)
        : Base(std::move(intrinsics)), stddevs_(std::move(stddevs)) {}

    // Were hidden after inheritance and now made visible again.
    using Base::Distort;
    using Base::intrinsics;
    using Base::K;
    using Base::Undistort;

    // Distort image coordinates as if the distorted image does not have the same K matrix as the undistorted image.
    template <typename T>
    void Distort(
        Eigen::Matrix3d const& K_undistorted,
        Eigen::Vector2f const& p_u,
        Eigen::Vector2f* p_d,
        Eigen::Matrix<T, 2, 1>* p_d_stddevs,
        const unsigned int num_threads = std::thread::hardware_concurrency()) const;

    // Undistort image coordinates as if the distorted image does not have the same K matrix as the undistorted image.
    template <typename T>
    void Undistort(
        Eigen::Matrix3d const& K_undistorted,
        Eigen::Vector2f const& p_d,
        Eigen::Vector2f* p_u,
        Eigen::Matrix<T, 2, 1>* p_u_stddevs,
        const unsigned int num_threads = std::thread::hardware_concurrency()) const;

protected:
    std::vector<double> stddevs_;
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
template <typename T>
void PinholeDistortionModelExtended<FL, SK, RA, TA, TP>::Distort(
    Eigen::Matrix3d const& K_undistorted,
    Eigen::Vector2f const& p_u,
    Eigen::Vector2f* p_d,
    Eigen::Matrix<T, 2, 1>* p_d_stddevs,
    const unsigned int num_threads) const {
    PinholeDistortionModelExtended dm(K_undistorted);
    Base::Distort(dm.intrinsics(), Base::intrinsics_, p_u, p_d);

    // The undistorted point initialized with the distorted one. In general,
    // initialization for gradient decent is important. Is this choice a problem?
    Eigen::Vector2d cp_u = p_u.template cast<double>();
    std::vector<double> ceres_distorted{p_d->x(), p_d->y()};
    std::vector<double> ceres_intrinsics = Base::intrinsics_;
    std::vector<double> ceres_weights = VecWeightedCostFunction::WeightsFromStddevs(stddevs_);

    ceres::Problem problem;
    ceres::CostFunction* cost_function_distort =
        PinholeDistortWithIntrinsicsCost<FL, SK, RA, TA, TP>::Create(dm.intrinsics(), cp_u);
    problem.AddResidualBlock(cost_function_distort, nullptr, ceres_intrinsics.data(), ceres_distorted.data());

    ceres::CostFunction* cost_function_intrinsics =
        VecWeightedCostFunction::Create<IdentityCostFunction<Base::kParametersIntrinsics>>(
            ceres_weights, Base::intrinsics_);
    problem.AddResidualBlock(cost_function_intrinsics, nullptr, ceres_intrinsics.data());

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);
    std::vector<std::pair<const double*, const double*>> covariance_blocks{
        {ceres_distorted.data(), ceres_distorted.data()}};
    covariance.Compute(covariance_blocks, &problem);

    *p_d_stddevs = GetStddevs2<T>(covariance, ceres_distorted.data());
    p_d->x() = static_cast<float>(ceres_distorted[0]);
    p_d->y() = static_cast<float>(ceres_distorted[1]);
}

// Undistort image coordinates as if the distorted image does not have the same K matrix as the undistorted image.
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
template <typename T>
void PinholeDistortionModelExtended<FL, SK, RA, TA, TP>::Undistort(
    Eigen::Matrix3d const& K_undistorted,
    Eigen::Vector2f const& p_d,
    Eigen::Vector2f* p_u,
    Eigen::Matrix<T, 2, 1>* p_u_stddevs,
    const unsigned int num_threads) const {
    PinholeDistortionModelExtended dm(K_undistorted);
    Base::Undistort(Base::intrinsics_, dm.intrinsics_, num_threads, p_d, p_u);

    Eigen::Vector2d cp_d = p_d.template cast<double>();
    std::vector<double> ceres_undistorted = {p_u->x(), p_u->y()};
    std::vector<double> ceres_intrinsics = Base::intrinsics_;
    std::vector<double> ceres_weights = VecWeightedCostFunction::WeightsFromStddevs(stddevs_);

    ceres::Problem problem;
    ceres::CostFunction* cost_function_undistort =
        PinholeUndistortWithIntrinsicsCost<FL, SK, RA, TA, TP>::Create(dm.intrinsics_, cp_d);
    problem.AddResidualBlock(cost_function_undistort, nullptr, ceres_intrinsics.data(), ceres_undistorted.data());

    ceres::CostFunction* cost_function_intrinsics =
        VecWeightedCostFunction::Create<IdentityCostFunction<Base::kParametersIntrinsics>>(
            ceres_weights, Base::intrinsics_);
    problem.AddResidualBlock(cost_function_intrinsics, nullptr, ceres_intrinsics.data());

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);
    std::vector<std::pair<const double*, const double*>> covariance_blocks{
        {ceres_undistorted.data(), ceres_undistorted.data()}};
    covariance.Compute(covariance_blocks, &problem);

    *p_u_stddevs = GetStddevs2<T>(covariance, ceres_undistorted.data());
    p_u->x() = static_cast<T>(ceres_undistorted[0]);
    p_u->y() = static_cast<T>(ceres_undistorted[1]);
}

}  // namespace nie

#endif  // NIE_CV_CALIB3D_PINHOLE_MODEL_EXTENDED_HPP
