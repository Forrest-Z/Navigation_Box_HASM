/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "weighted_cost_function.hpp"

namespace nie {

VecWeightedCostFunction::VecWeightedCostFunction(
    std::vector<double> weights, ceres::CostFunction* wrapped_cost_function, ceres::Ownership ownership)
    : weights_(std::move(weights)), wrapped_cost_function_(wrapped_cost_function), ownership_(ownership) {
    set_num_residuals(wrapped_cost_function_->num_residuals());
    *mutable_parameter_block_sizes() = wrapped_cost_function_->parameter_block_sizes();

    // Sanity-check the weight count.
    CHECK_EQ(wrapped_cost_function_->num_residuals(), weights_.size());
}

VecWeightedCostFunction::~VecWeightedCostFunction() {
    if (ownership_ != ceres::TAKE_OWNERSHIP) {
        wrapped_cost_function_.release();
    }
}

ceres::CostFunction* VecWeightedCostFunction::Create(
    std::vector<double> weights, ceres::CostFunction* wrapped_cost_function) {
    return new VecWeightedCostFunction(std::move(weights), wrapped_cost_function, ceres::TAKE_OWNERSHIP);
}

std::vector<double> VecWeightedCostFunction::WeightsFromStddevs(std::vector<double> const& stddevs) {
    std::vector<double> weights(stddevs.size());
    // ceres::VectorRef(weights.data(), weights.size()) = ceres::ConstVectorRef(stddevs.data(),
    // stddevs.size()).array().inverse().matrix();
    for (std::size_t i = 0; i < stddevs.size(); ++i) weights[i] = 1.0 / stddevs[i];

    return weights;
}

std::vector<double> VecWeightedCostFunction::WeightsFromVariances(std::vector<double> const& variances) {
    std::vector<double> weights(variances.size());
    // ceres::VectorRef(weights.data(), weights.size()) = ceres::ConstVectorRef(stddevs.data(),
    // stddevs.size()).array().inverse().sqrt().matrix();
    for (std::size_t i = 0; i < variances.size(); ++i) weights[i] = std::sqrt(1.0 / variances[i]);

    return weights;
}

bool VecWeightedCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    bool success = wrapped_cost_function_->Evaluate(parameters, residuals, jacobians);

    if (!success) {
        return false;
    }

    for (int i = 0; i < wrapped_cost_function_->num_residuals(); ++i) {
        residuals[i] = residuals[i] * weights_[i];
    }

    if (jacobians) {
        for (int r = 0; r < wrapped_cost_function_->num_residuals(); ++r) {
            for (std::size_t i = 0; i < wrapped_cost_function_->parameter_block_sizes().size(); ++i) {
                if (jacobians[i]) {
                    int parameter_block_size = wrapped_cost_function_->parameter_block_sizes()[i];

                    ceres::VectorRef jacobian_row(jacobians[i] + r * parameter_block_size, parameter_block_size, 1);

                    jacobian_row *= weights_[r];
                }
            }
        }
    }

    return true;
}

MatWeightedCostFunction::MatWeightedCostFunction(
    std::vector<double> weights, ceres::CostFunction* wrapped_cost_function, ceres::Ownership ownership)
    : weights_(std::move(weights)), wrapped_cost_function_(wrapped_cost_function), ownership_(ownership) {
    set_num_residuals(wrapped_cost_function_->num_residuals());
    *mutable_parameter_block_sizes() = wrapped_cost_function_->parameter_block_sizes();

    // Sanity-check the weight count.
    CHECK_EQ(wrapped_cost_function_->num_residuals() * wrapped_cost_function_->num_residuals(), weights_.size());
}

MatWeightedCostFunction::~MatWeightedCostFunction() {
    if (ownership_ != ceres::TAKE_OWNERSHIP) {
        wrapped_cost_function_.release();
    }
}

ceres::CostFunction* MatWeightedCostFunction::Create(
    std::vector<double> weights, ceres::CostFunction* wrapped_cost_function) {
    return new MatWeightedCostFunction(std::move(weights), wrapped_cost_function, ceres::TAKE_OWNERSHIP);
}

bool MatWeightedCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    bool success = wrapped_cost_function_->Evaluate(parameters, residuals, jacobians);

    if (!success) {
        return false;
    }

    const int num_residuals = wrapped_cost_function_->num_residuals();
    ceres::ConstMatrixRef w(weights_.data(), num_residuals, num_residuals);

    std::vector<double> residuals_copy(num_residuals);
    std::copy(residuals, residuals + num_residuals, residuals_copy.data());

    ceres::VectorRef residuals_in(residuals_copy.data(), num_residuals);
    ceres::VectorRef residuals_out(residuals, num_residuals);

    residuals_out = w * residuals_in;

    if (jacobians) {
        for (std::size_t i = 0; i < wrapped_cost_function_->parameter_block_sizes().size(); ++i) {
            const int num_parameters = wrapped_cost_function_->parameter_block_sizes()[i];
            const int num_elements = num_residuals * num_parameters;

            std::vector<double> jacobian_copy(num_elements);
            std::copy(jacobians[i], jacobians[i] + num_elements, jacobian_copy.data());

            ceres::MatrixRef jacobian_in(jacobian_copy.data(), num_residuals, num_parameters);
            ceres::MatrixRef jacobian_out(jacobians[i], num_residuals, num_parameters);

            jacobian_out = w * jacobian_in;
        }
    }

    return true;
}

}  // namespace nie