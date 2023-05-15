/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/cv/calib3d/pinhole_distortion_model_extended.hpp>
#include <nie/cv/ceres/identity_cost_function.hpp>
#include <nie/formats/calib3d/calibrated_mono_parameters.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>

#include "pinhole_distort_weighted_cost.hpp"
#include "pinhole_undistort_weighted_cost.hpp"

using PinholeDistortionModelExtended = nie::PinholeDistortionModelExtended<
    nie::ParameterFocalLength::XY_F,
    nie::ParameterSkew::NO_SKEW,
    nie::ParameterDistortionRadial::K3,
    nie::ParameterDistortionTangential::P2,
    nie::ParameterDistortionThinPrism::NO_DISTORTION>;

using PinholeDistortWeightedCost = nie::PinholeDistortWeightedCost<
    nie::ParameterFocalLength::XY_F,
    nie::ParameterSkew::NO_SKEW,
    nie::ParameterDistortionRadial::K3,
    nie::ParameterDistortionTangential::P2,
    nie::ParameterDistortionThinPrism::NO_DISTORTION>;

using PinholeUndistortWeightedCost = nie::PinholeUndistortWeightedCost<
    nie::ParameterFocalLength::XY_F,
    nie::ParameterSkew::NO_SKEW,
    nie::ParameterDistortionRadial::K3,
    nie::ParameterDistortionTangential::P2,
    nie::ParameterDistortionThinPrism::NO_DISTORTION>;

template <typename T>
void CustomDistort(
    std::vector<double> const& intrinsics,
    std::vector<double> const& stddevs,
    Eigen::Matrix3d const& K_undistorted,
    Eigen::Vector2f const& p_u,
    Eigen::Vector2f* p_d,
    Eigen::Matrix<T, 2, 1>* p_d_stddevs,
    const unsigned int num_threads) {
    PinholeDistortionModelExtended de(intrinsics, stddevs);
    PinholeDistortionModelExtended dm(K_undistorted);
    de.Distort(dm.intrinsics(), intrinsics, p_u, p_d);

    // The undistorted point initialized with the distorted one. In general,
    // initialization for gradient decent is important. Is this choice a problem?
    constexpr std::size_t kParametersIntrinsics = PinholeDistortionModelExtended::kParametersIntrinsics;
    std::vector<double> ceres_intrinsics = intrinsics;
    std::vector<double> ceres_weights(kParametersIntrinsics);

    for (std::size_t i = 0; i < kParametersIntrinsics; ++i) ceres_weights[i] = 1.0 / stddevs[i];

    Eigen::Vector2d cp_u = p_u.template cast<double>();
    Eigen::Vector2d cp_d = p_d->template cast<double>();

    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        PinholeDistortWeightedCost::Create(dm.intrinsics(), intrinsics, ceres_weights, cp_u);
    problem.AddResidualBlock(cost_function, nullptr, ceres_intrinsics.data(), cp_d.data());

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);
    std::vector<std::pair<const double*, const double*> > covariance_blocks{{cp_d.data(), cp_d.data()}};
    covariance.Compute(covariance_blocks, &problem);

    std::vector<double> covariance_matrix(4);
    covariance.GetCovarianceBlock(cp_d.data(), cp_d.data(), covariance_matrix.data());

    *p_d = cp_d.template cast<float>();
    p_d_stddevs->x() = static_cast<T>(std::sqrt(covariance_matrix[0]));
    p_d_stddevs->y() = static_cast<T>(std::sqrt(covariance_matrix[3]));
}

template <typename T>
void CustomUndistort(
    std::vector<double> const& intrinsics,
    std::vector<double> const& stddevs,
    Eigen::Matrix3d const& K_undistorted,
    Eigen::Vector2f const& p_d,
    Eigen::Vector2f* p_u,
    Eigen::Matrix<T, 2, 1>* p_u_stddevs,
    const unsigned int num_threads) {
    PinholeDistortionModelExtended de(intrinsics, stddevs);
    PinholeDistortionModelExtended dm(K_undistorted);
    de.Undistort(intrinsics, dm.intrinsics(), num_threads, p_d, p_u);

    constexpr std::size_t kParametersIntrinsics = PinholeDistortionModelExtended::kParametersIntrinsics;
    std::vector<double> ceres_intrinsics = intrinsics;
    std::vector<double> ceres_weights(kParametersIntrinsics);

    for (std::size_t i = 0; i < kParametersIntrinsics; ++i) ceres_weights[i] = 1.0 / stddevs[i];

    Eigen::Vector2d cp_d = p_d.template cast<double>();
    Eigen::Vector2d cp_u = p_u->template cast<double>();

    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        PinholeUndistortWeightedCost::Create(intrinsics, ceres_weights, dm.intrinsics(), cp_d);
    problem.AddResidualBlock(cost_function, nullptr, ceres_intrinsics.data(), cp_u.data());

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);
    std::vector<std::pair<const double*, const double*> > covariance_blocks{{cp_u.data(), cp_u.data()}};
    covariance.Compute(covariance_blocks, &problem);

    std::vector<double> covariance_matrix(4);
    covariance.GetCovarianceBlock(cp_u.data(), cp_u.data(), covariance_matrix.data());

    *p_u = cp_u.template cast<float>();
    p_u_stddevs->x() = static_cast<T>(std::sqrt(covariance_matrix[0]));
    p_u_stddevs->y() = static_cast<T>(std::sqrt(covariance_matrix[3]));
}

class PinholeDistortionModelExtendedTest : public ::testing::Test {
protected:
    PinholeDistortionModelExtendedTest()
        : cp_(nie::io::CalibratedMonoParameters::Read(
              "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/intrinsics_mono.json")),
          rp_(nie::io::RectifiedCameraParameters::Read(
              "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/rectified_intrinsics_mono_vector.json")) {}

    nie::io::CalibratedMonoParameters cp_;
    nie::io::RectifiedCameraParameters rp_;
};

TEST_F(PinholeDistortionModelExtendedTest, Distort) {
    Eigen::Matrix<double, 9, 1> lens_stddevs = cp_.lens().cov.diagonal().cwiseSqrt();
    std::vector<double> lens_stddevs_vec{lens_stddevs.data(), lens_stddevs.data() + lens_stddevs.size()};

    PinholeDistortionModelExtended model_extended(cp_.lens().intrinsics, lens_stddevs_vec);

    Eigen::Matrix3d K = rp_.K;
    Eigen::Vector2f u(0, 0);
    Eigen::Vector2f dv, ds1, ds2;
    Eigen::Vector2f stddevs1, stddevs2;

    // Same as the base class
    model_extended.Distort(K, u, &dv);
    // The extended version
    model_extended.Distort(K, u, &ds1, &stddevs1);
    // Custom version that should be equal but using only a single cost functor instead of
    // compound classes (the old class)
    CustomDistort(cp_.lens().intrinsics, lens_stddevs_vec, K, u, &ds2, &stddevs2, 8);

    ASSERT_NEAR((dv - ds1).norm(), 0.0, 0.00000000000001);
    ASSERT_NEAR((dv - ds2).norm(), 0.0, 0.00000000000001);
    ASSERT_NEAR((stddevs1 - stddevs2).norm(), 0.0, 0.00000000000001);
}

TEST_F(PinholeDistortionModelExtendedTest, Undistort) {
    Eigen::Matrix<double, 9, 1> lens_stddevs = cp_.lens().cov.diagonal().cwiseSqrt();
    std::vector<double> lens_stddevs_vec{lens_stddevs.data(), lens_stddevs.data() + lens_stddevs.size()};

    PinholeDistortionModelExtended model_extended(cp_.lens().intrinsics, lens_stddevs_vec);

    Eigen::Matrix3d K = rp_.K;
    Eigen::Vector2f d(0, 0);
    Eigen::Vector2f uv, us1, us2;
    Eigen::Vector2f stddevs1, stddevs2;

    // Same as the base class
    model_extended.Undistort(K, d, &uv);
    // The extended version
    model_extended.Undistort(K, d, &us1, &stddevs1);
    // Custom version that should be equal but using only a single cost functor instead of
    // compound classes (the old class)
    CustomUndistort(cp_.lens().intrinsics, lens_stddevs_vec, K, d, &us2, &stddevs2, 8);

    ASSERT_NEAR((uv - us1).norm(), 0.0, 0.00000000000001);
    ASSERT_NEAR((uv - us2).norm(), 0.0, 0.00000000000001);
    ASSERT_NEAR((stddevs1 - stddevs2).norm(), 0.0, 0.00000000000001);
}
