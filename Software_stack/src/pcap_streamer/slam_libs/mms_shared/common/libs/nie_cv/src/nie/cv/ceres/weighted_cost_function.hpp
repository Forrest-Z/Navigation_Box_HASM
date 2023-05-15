/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_WEIGHTED_COST_FUNCTION_HPP
#define NIE_CV_CERES_WEIGHTED_COST_FUNCTION_HPP

#include <ceres/ceres.h>

namespace nie {

// Modeled after the ceres::ConditionedCostFunction from ceres it self.
// The main difference is that you don't have to input an array of cost functions
// which does the re-weighting for each residual (too general because we only want
// to scale and not apply an arbitrary function and there is too much overhead).
//
// The simple use case is to apply weights based on the variances of the
// residuals to obtain the weighted normal equations. These weights should be
// 1.0 / variance = sqrt(1.0 / variance^2)
class VecWeightedCostFunction final : public ceres::CostFunction {
public:
    VecWeightedCostFunction(
        std::vector<double> weights, ceres::CostFunction* wrapped_cost_function, ceres::Ownership ownership);

    ~VecWeightedCostFunction() final;

    static ceres::CostFunction* Create(std::vector<double> weights, ceres::CostFunction* wrapped_cost_function);
    template <typename CeresCostFunction, typename... Args>
    static ceres::CostFunction* Create(std::vector<double> weights, Args... args);
    static std::vector<double> WeightsFromStddevs(std::vector<double> const& stddevs);
    static std::vector<double> WeightsFromVariances(std::vector<double> const& variances);

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const final;

private:
    std::vector<double> weights_;
    std::unique_ptr<ceres::CostFunction> wrapped_cost_function_;
    ceres::Ownership ownership_;
};

template <typename CeresCostFunction, typename... Args>
ceres::CostFunction* VecWeightedCostFunction::Create(std::vector<double> weights, Args... args) {
    return new VecWeightedCostFunction(std::move(weights), new CeresCostFunction(args...), ceres::TAKE_OWNERSHIP);
}

class MatWeightedCostFunction final : public ceres::CostFunction {
public:
    MatWeightedCostFunction(
        std::vector<double> weights, ceres::CostFunction* wrapped_cost_function, ceres::Ownership ownership);

    ~MatWeightedCostFunction() final;

    static ceres::CostFunction* Create(std::vector<double> weights, ceres::CostFunction* wrapped_cost_function);
    template <typename CeresCostFunction, typename... Args>
    static ceres::CostFunction* Create(std::vector<double> weights, Args... args);

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const final;

private:
    std::vector<double> weights_;
    std::unique_ptr<ceres::CostFunction> wrapped_cost_function_;
    ceres::Ownership ownership_;
};

template <typename CeresCostFunction, typename... Args>
ceres::CostFunction* MatWeightedCostFunction::Create(std::vector<double> weights, Args... args) {
    return new MatWeightedCostFunction(std::move(weights), new CeresCostFunction(args...), ceres::TAKE_OWNERSHIP);
}

}  // namespace nie

#endif  // NIE_CV_CERES_WEIGHTED_COST_FUNCTION_HPP
