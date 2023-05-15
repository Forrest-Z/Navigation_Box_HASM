/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_IDENTITY_COST_FUNCTION_HPP
#define NIE_CV_CERES_IDENTITY_COST_FUNCTION_HPP

#include <ceres/ceres.h>

namespace nie {

// Simple identity cost function: || residual = x - x_constant || ^ 2
// The Jacobian equals the identity matrix.
// It is only allowed to use a single parameter block within a identity cost function.
// Can be used for error propagation.
template <int kNumResiduals>
class IdentityCostFunction final : public ceres::SizedCostFunction<kNumResiduals, kNumResiduals> {
public:
    explicit IdentityCostFunction(std::vector<double> values);

    static ceres::CostFunction* Create(std::vector<double> values);

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const final;

private:
    std::vector<double> values_;
};

template <int kNumResiduals>
IdentityCostFunction<kNumResiduals>::IdentityCostFunction(std::vector<double> values) : values_(std::move(values)) {
    CHECK_EQ(values_.size(), kNumResiduals);
}

template <int kNumResiduals>
ceres::CostFunction* IdentityCostFunction<kNumResiduals>::Create(std::vector<double> values) {
    return new IdentityCostFunction<kNumResiduals>(values);
}

template <int kNumResiduals>
bool IdentityCostFunction<kNumResiduals>::Evaluate(
    double const* const* parameters, double* residuals, double** jacobians) const {
    for (std::size_t r = 0; r < kNumResiduals; ++r) residuals[r] = parameters[0][r] - values_[r];

    if (jacobians) {
        ceres::MatrixRef jacobian_matrix(jacobians[0], kNumResiduals, kNumResiduals);
        jacobian_matrix.setIdentity();
    }

    return true;
}

}  // namespace nie

#endif  // NIE_CV_CERES_IDENTITY_COST_FUNCTION_HPP
