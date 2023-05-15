/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/core/ceres/error_propagation.hpp>

#include "extrinsics_adapter.hpp"

template <int N>
class IdentityFunction {
public:
    template <typename T>
    bool operator()(const T* const x, T* y) const {
        std::copy(x, x + N, y);

        return true;
    }
};

TEST(ErrorPropagationTest, Identity) {
    cv::Matx33d covar_x = cv::Matx33d::eye();
    cv::Matx33d covar_y = cv::Matx33d::zeros();
    cv::Point3d x(0.0, 1.0, 0.0);
    cv::Point3d y;

    nie::AutoDiffErrorPropagator<IdentityFunction<3>, 3, 3>().Propagate(covar_x.val, &x.x, covar_y.val, &y.x);

    ASSERT_NEAR(cv::norm(covar_x - covar_y), 0.0, std::numeric_limits<double>::epsilon());
    ASSERT_NEAR(cv::norm(x - y), 0.0, std::numeric_limits<double>::epsilon());
}

class ExtrinsicsInvertFunctionSplit {
public:
    template <typename T>
    bool operator()(const T* const t, const T* const r, T* y) const {
        T x[6] = {t[0], t[1], t[2], r[0], r[1], r[2]};

        nie::ExtrinsicsAdapter<T const>(x).Invert(y);

        return true;
    }
};

const std::vector<double> kCovarX{
    2.33E-08,  -2.59E-10, 7.40E-09,  -1.06E-09, 4.08E-08,  2.02E-09,  -2.59E-10, 2.71E-08,  -1.78E-09,
    -3.16E-08, -1.70E-09, -7.03E-09, 7.40E-09,  -1.78E-09, 1.20E-07,  -3.80E-08, 3.93E-08,  -1.21E-09,
    -1.06E-09, -3.16E-08, -3.80E-08, 4.76E-07,  2.57E-09,  2.42E-08,  4.08E-08,  -1.70E-09, 3.93E-08,
    2.57E-09,  3.60E-07,  1.04E-08,  2.02E-09,  -7.03E-09, -1.21E-09, 2.42E-08,  1.04E-08,  1.34E-08};

const std::vector<double> kVariablesX{-3.6392037959780293e-01,
                                      -1.3521162215099894e-01,
                                      7.9411396553887448e-01,
                                      1.8847019533443058e-03,
                                      7.8828386072849768e-02,
                                      4.0627375105776645e-02};

// The data comes from two mono calibrations that used the same picture. But one
// used a square size 1.0 for the checkerboard square and the other the value
// below. This means that positional related values are scaled too.
const double kScalar = 2.9629999771714211e-02;

const cv::Vec6d kCorrectVariablesY{1.4528281909298553e+01 * kScalar,
                                   3.9687977098631393e+00 * kScalar,
                                   -2.5751652338477172e+01 * kScalar,
                                   -1.8858039617921135e-03,
                                   -7.8827788741788266e-02,
                                   -4.0627542211538151e-02};

// The full covariance matrix was never available for this dataset. Testing for
// the diagonal seems enough (should be wrong too if the math was wrong).
const cv::Vec6d kCorrectSigmasY{1.4174705127293390e-02 * kScalar,
                                1.7815126392526005e-02 * kScalar,
                                1.2973062524455908e-02 * kScalar,
                                6.9022184658605124e-04,
                                5.9963056829973884e-04,
                                1.1595369074916944e-04};

TEST(ErrorPropagationTest, SingleParameterBlock) {
    cv::Matx66d covar_y;
    cv::Vec6d y;

    nie::AutoDiffErrorPropagator<nie::ExtrinsicsInvertFunction, 6, 6>().Propagate(
        kCovarX.data(), kVariablesX.data(), covar_y.val, y.val);

    cv::Vec6d sigmas_y;

    for (int i = 0; i < 6; ++i) sigmas_y[i] = std::sqrt(covar_y(i, i));

    ASSERT_NEAR(cv::norm(kCorrectSigmasY - sigmas_y), 0.0, 0.000001);
    ASSERT_NEAR(cv::norm(kCorrectVariablesY - y), 0.0, 0.00001);
}

TEST(ErrorPropagationTest, MultipleParameterBlocks) {
    cv::Matx66d covar_y;
    cv::Vec6d y;

    std::vector<double const*> x_refs{&kVariablesX[0], &kVariablesX[3]};

    nie::AutoDiffErrorPropagator<ExtrinsicsInvertFunctionSplit, 6, 3, 3>().Propagate(
        kCovarX.data(), x_refs.data(), covar_y.val, y.val);

    cv::Vec6d sigmas_y;

    for (int i = 0; i < 6; ++i) sigmas_y[i] = std::sqrt(covar_y(i, i));

    ASSERT_NEAR(cv::norm(kCorrectSigmasY - sigmas_y), 0.0, 0.000001);
    ASSERT_NEAR(cv::norm(kCorrectVariablesY - y), 0.0, 0.00001);
}