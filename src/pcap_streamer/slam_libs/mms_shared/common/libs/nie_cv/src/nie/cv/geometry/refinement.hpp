/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_GEOMETRY_REFINEMENT_HPP
#define NIE_CV_GEOMETRY_REFINEMENT_HPP

#include <Eigen/Dense>

namespace nie {

namespace detail {

// Contains some constants required for finding the coefficients of a 2D quadratic function.
// The 2D quadratic function is defined as f(x, y) = ax^2 + by^2 + cxy + dx + ey + f
// These coefficients are: a, b, c, d, e, and f.
// They are found by solving the linearly. With 6 unknowns we need a minimum of 6 samples. However,
// we assume there are always 9 given that we typically sample for pixels and want to refine the center.
class QuadraticRefinementA {
public:
    QuadraticRefinementA() {
        // For each of the 9 samples we write out the equations.
        // Column major
        // clang-format off
        Eigen::Matrix<float, 9, 6> A;
        A <<
            1, 1, 1,-1,-1, 1,
            1, 0, 0,-1, 0, 1,
            1, 1,-1,-1, 1, 1,
            0, 1, 0, 0,-1, 1,
            0, 0, 0, 0, 0, 1,
            0, 1, 0, 0, 1, 1,
            1, 1,-1, 1,-1, 1,
            1, 0, 0, 1, 0, 1,
            1, 1, 1, 1, 1, 1;
        // clang-format on
        A_t = A.transpose();
        solver = (A_t * A).colPivHouseholderQr();
    }

    Eigen::Matrix<float, 6, 9> A_t;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, 6, 6>> solver;
};

}  // namespace detail

// Assumes 3x3 sample data in column major order:
//
// s(0, 0): x = -1, y = -1,     s(0, 1): x =  0, y = -1,    s(0, 2): x =  1, y = -1
// s(1, 0): x = -1, y =  0,     s(1, 1): x =  0, y =  0,    s(1, 2): x =  1, y =  0
// s(2, 0): x = -1, y =  1,     s(2, 1): x =  0, y =  1,    s(2, 2): x =  1, y =  1
//
// A vector is also allowed:
// s(0): x = -1, y = -1,
// s(1): x = -1, y =  0,
// s(2): x = -1, y =  1,
// s(3): x =  0, y = -1,
// s(4): x =  0, y =  0,
// s(5): x =  0, y =  1,
// ...
//
// For more information: https://en.wikipedia.org/wiki/Quadratic_function#Bivariate_(two_variable)_quadratic_function
template <typename Derived>
bool RefineQuadratic2D(Eigen::MatrixBase<Derived> const& samples, Eigen::Vector2f* refined) {
    static_assert(std::is_same<typename Derived::Scalar, float>::value, "INPUT_IS_EXPECTED_TO_BE_FLOAT");
    static_assert(
        Derived::RowsAtCompileTime * Derived::ColsAtCompileTime == 9, "COMPILE_TIME_SIZE_OF_SAMPLES_SHOULD_BE_9");
    static detail::QuadraticRefinementA A;
    Eigen::Map<Eigen::Matrix<float, 9, 1> const> y(samples.derived().data());
    Eigen::Matrix<float, 6, 1> x = A.solver.solve(A.A_t * y);

    float d = 4.0f * x(0) * x(1) - x(2) * x(2);

    // If d <= 0 it is a surface shape that we can't use.
    if (d < std::numeric_limits<float>::epsilon()) return false;

    float fx = (x(2) * x(4) - 2.0f * x(1) * x(3)) / d;
    float fy = (x(2) * x(3) - 2.0f * x(0) * x(4)) / d;

    (*refined)(0) += fx;
    (*refined)(1) += fy;

    return true;
}

}  // namespace nie

#endif  // NIE_CV_GEOMETRY_REFINEMENT_HPP
