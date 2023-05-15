/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/cv/ceres/identity_cost_function.hpp>
#include <nie/cv/ceres/weighted_cost_function.hpp>

TEST(WeightedCostFunctionTest, VecWeighted) {
    std::vector<double> residuals(3, 0.0);
    std::vector<double> jacobians(9, 0.0);
    std::vector<double> weights = {2.0, 0.5, 0.25};
    std::vector<double> parameters{1.0, 1.0, 1.0};
    std::vector<double> values{0.0, 0.0, 0.0};

    nie::VecWeightedCostFunction f(
        weights, nie::IdentityCostFunction<3>::Create(values), ceres::Ownership::TAKE_OWNERSHIP);

    std::vector<double*> ref_parameters{&parameters[0]};
    std::vector<double*> ref_jacobians{&jacobians[0]};

    f.Evaluate(ref_parameters.data(), residuals.data(), ref_jacobians.data());

    std::vector<double> e_residuals = {2.0, 0.5, 0.25};
    std::vector<double> e_jacobians = {2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.25};

    for (std::size_t i = 0; i < e_residuals.size(); ++i)
        ASSERT_NEAR(e_residuals[i], residuals[i], std::numeric_limits<double>::epsilon());

    for (std::size_t i = 0; i < e_jacobians.size(); ++i)
        ASSERT_NEAR(e_jacobians[i], jacobians[i], std::numeric_limits<double>::epsilon());
}

TEST(WeightedCostFunctionTest, MatWeighted) {
    std::vector<double> residuals(3, 0.0);
    std::vector<double> jacobians(9, 0.0);
    std::vector<double> weights = {2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.25};
    std::vector<double> parameters{1.0, 1.0, 1.0};
    std::vector<double> values{0.0, 0.0, 0.0};

    nie::MatWeightedCostFunction f(
        weights, nie::IdentityCostFunction<3>::Create(values), ceres::Ownership::TAKE_OWNERSHIP);

    std::vector<double*> ref_parameters{&parameters[0]};
    std::vector<double*> ref_jacobians{&jacobians[0]};

    f.Evaluate(ref_parameters.data(), residuals.data(), ref_jacobians.data());

    std::vector<double> e_residuals = {2.0, 0.5, 0.25};
    std::vector<double> e_jacobians = {2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.25};

    for (std::size_t i = 0; i < e_residuals.size(); ++i)
        ASSERT_NEAR(e_residuals[i], residuals[i], std::numeric_limits<double>::epsilon());

    for (std::size_t i = 0; i < e_jacobians.size(); ++i)
        ASSERT_NEAR(e_jacobians[i], jacobians[i], std::numeric_limits<double>::epsilon());
}
