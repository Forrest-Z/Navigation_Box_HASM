/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <ceres/ceres.h>

namespace nie {

// NOTE: Input as double pointers due to ceres.

// Propagates the errors described by the covariance matrix of the input
// parameters to those of the function (and thus its output).
//
//      Covar(f(x)) = J Covar(xx^T) J^T,
//
// where J equals the jacobian of f(x) with respect to x.
//
// Assumes the covariance matrix of the parameters has the same ordering
// as the parameter blocks.
//
// Same template arguments as ceres::AutoDiffCostFunction.
template <
        typename Functor,  // Function type.
        int kNumOutputs,   // Number of function outputs.
        int N0,            // Number of parameters in block 0.
        int N1 = 0,        // Number of parameters in block 1.
        int N2 = 0,        // Number of parameters in block 2.
        int N3 = 0,        // Number of parameters in block 3.
        int N4 = 0,        // Number of parameters in block 4.
        int N5 = 0,        // Number of parameters in block 5.
        int N6 = 0,        // Number of parameters in block 6.
        int N7 = 0,        // Number of parameters in block 7.
        int N8 = 0,        // Number of parameters in block 8.
        int N9 = 0         // Number of parameters in block 9.
        >
class AutoDiffErrorPropagator {
public:
    template <typename... Args>
    AutoDiffErrorPropagator(Args... args)
        : auto_diff_f_(new Functor(args...)),
          parameter_block_sizes_(auto_diff_f_.parameter_block_sizes()),
          parameter_count_(GetParameterCount()) {
        buffer_jacobians_.resize(parameter_block_sizes_.size());
        jacobian_references_.resize(parameter_block_sizes_.size());

        for (std::size_t i = 0; i < buffer_jacobians_.size(); ++i) {
            buffer_jacobians_[i].resize(static_cast<std::size_t>(parameter_block_sizes_[i] * kNumOutputs));
            jacobian_references_[i] = buffer_jacobians_[i].data();
        }
    }

    bool Propagate(
            double const* const covariance_x, double const* const* x, double* const covariance_y, double* const y) {
        return Propagate(covariance_x, x, covariance_y, y, jacobian_references_.data());
    }

    bool Propagate(double const* const covariance_x, double const* x, double* const covariance_y, double* const y) {
        // ceres works with parameter blocks. This means groups of input
        // variables. Here we are friendly enough to not bother the user
        // with this in case he/she only has a single parameter block.
        std::vector<double const*> p_x_temp{x};

        return Propagate(covariance_x, p_x_temp.data(), covariance_y, y, jacobian_references_.data());
    }

private:
    int GetParameterCount() {
        int count = 0;

        for (std::size_t i = 0; i < parameter_block_sizes_.size(); ++i) count += parameter_block_sizes_[i];

        return count;
    }

    bool Propagate(
            double const* const covariance_x,
            double const* const* x,
            double* const covariance_y,
            double* const y,
            double** jacobians_y_over_x) {
        // Get function output and jacobian
        bool success = auto_diff_f_.Evaluate(x, y, jacobians_y_over_x);

        // *************************************************************************************
        // Everything below calculates the covariance of the function (of result y)
        // *************************************************************************************

        ceres::ConstMatrixRef covariance_parameters_matrix(covariance_x, parameter_count_, parameter_count_);
        ceres::MatrixRef covariance_out_matrix(covariance_y, kNumOutputs, kNumOutputs);
        covariance_out_matrix.setZero();

        int covariance_x_offset_col = 0;

        // Because the jacobian is split into parameter block amount of parts, the calculation of
        // the covariance of y is partitioned in jacobian blocks * jacobian blocks amount of
        // sub blocks.

        // Goes over all covariance columns
        for (std::size_t j = 0; j < parameter_block_sizes_.size(); ++j) {
            int const block_size_j = auto_diff_f_.parameter_block_sizes()[j];
            std::vector<double> left_j(static_cast<std::size_t>(kNumOutputs * block_size_j));
            ceres::MatrixRef left_j_matrix(left_j.data(), kNumOutputs, block_size_j);
            left_j_matrix.setZero();

            int covariance_x_offset_row = 0;

            // Goes over all covariance rows
            for (std::size_t i = 0; i < parameter_block_sizes_.size(); ++i) {
                int const block_size_i = auto_diff_f_.parameter_block_sizes()[i];
                ceres::ConstMatrixRef jacobian_matrix(jacobians_y_over_x[i], kNumOutputs, block_size_i);
                left_j_matrix += jacobian_matrix *
                                 covariance_parameters_matrix.block(
                                         covariance_x_offset_row, covariance_x_offset_col, block_size_i, block_size_j);
                covariance_x_offset_row += block_size_i;
            }

            ceres::ConstMatrixRef jacobian_matrix(jacobians_y_over_x[j], kNumOutputs, block_size_j);
            covariance_out_matrix += left_j_matrix * jacobian_matrix.transpose();
            covariance_x_offset_col += block_size_j;
        }

        return success;
    }

    ceres::AutoDiffCostFunction<Functor, kNumOutputs, N0, N1, N2, N3, N4, N5, N6, N7, N8, N9> auto_diff_f_;
    std::vector<int> const& parameter_block_sizes_;
    int const parameter_count_;
    // Buffer and references to the buffer (for ceres).
    // The buffer is kept here so the user is not annoyed by creating one (for now).
    std::vector<std::vector<double>> buffer_jacobians_;
    std::vector<double*> jacobian_references_;
};

}  // namespace nie
