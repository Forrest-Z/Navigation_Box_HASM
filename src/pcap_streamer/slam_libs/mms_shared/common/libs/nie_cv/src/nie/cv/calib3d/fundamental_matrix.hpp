/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_CALIB3D_FUNDAMENTAL_MATRIX_HPP
#define NIE_CV_CALIB3D_FUNDAMENTAL_MATRIX_HPP

#include <cassert>
#include <vector>

#include <opencv2/core.hpp>

#include "normalization.hpp"

// OpenCV already contains cv::findFundamentalMatrix.
// In some cases the result of that one seems worse which probably depends
// on some settings. The 8-point algorithm gives created here gives
// the exact same results as the OpenCV one

namespace nie {

// Computes the Fundamental matrix from at least 8 point correspondences
template <typename T>
cv::Matx33d EstimateFundamentalMatrix8Point(
    std::vector<cv::Point_<T>> const& points_a, std::vector<cv::Point_<T>> const& points_b) {
    assert(points_a.size() == points_b.size());

    // Find the solution to A^tAx=0 instead of Ax=0 as a speed optimization.
    cv::Matx<double, 9, 9> A;
    // A = A^tA can be created as a combination of outer products to reduce memory.
    for (std::size_t i = 0; i < points_a.size(); ++i) {
        // Implicit cast to double from T.
        cv::Point2d point_a = points_a[i];
        cv::Point2d point_b = points_b[i];

        cv::Vec<double, 9> row{// One point pair results in a single equation based on b^tFa = 0
                               // b.x*a.x*F_11 + ... +
                               point_b.x * point_a.x,
                               point_b.x * point_a.y,
                               point_b.x,
                               point_b.y * point_a.x,
                               point_b.y * point_a.y,
                               point_b.y,
                               point_a.x,
                               point_a.y,
                               1.0};

        A += row * row.t();
    }

    cv::SVD svd;
    svd(A, cv::SVD::MODIFY_A);

    // This is the linear solution to F in row form. However, typically it isn't singular and of rank 2.
    cv::Matx33d F(svd.vt.row(8).ptr<double>());
    // return F;

    // Make F singular and of rank 2 by setting the smallest eigen value to 0 and recomputing it. This method
    // minimizes the Frobenius norm between F_row and the resulting F.
    // Practically speaking this makes sure there are clean epipoles.
    cv::Vec3d w;
    cv::Matx33d u;
    cv::Matx33d vt;

    cv::SVD::compute(F, w, u, vt, cv::SVD::MODIFY_A);
    w[2] = 0.0;

    return u * cv::Matx33d::diag(w) * vt;
}

template <typename T>
cv::Matx33d EstimateFundamentalMatrixNormalized8Point(
    std::vector<cv::Point_<T>> const& points_a, std::vector<cv::Point_<T>> const& points_b) {
    assert(points_a.size() == points_b.size());

    std::vector<cv::Point2d> normalized_points_a(points_a.size()), normalized_points_b(points_b.size());

    cv::Matx33d n_a = ComputeNormalizationMatrix(points_a);
    cv::Matx33d n_b = ComputeNormalizationMatrix(points_b);

    for (std::size_t i = 0; i < points_a.size(); ++i) {
        // Implicit cast to double from T.
        cv::Point2d point_a = points_a[i];
        cv::Point2d point_b = points_b[i];

        // There is no use to apply the entire matrix since not every element is used.
        normalized_points_a[i].x = n_a(0, 0) * point_a.x + n_a(0, 2);
        normalized_points_a[i].y = n_a(1, 1) * point_a.y + n_a(1, 2);

        normalized_points_b[i].x = n_b(0, 0) * point_b.x + n_b(0, 2);
        normalized_points_b[i].y = n_b(1, 1) * point_b.y + n_b(1, 2);
    }

    // Return F that does not require normalized points.
    return n_b.t() * EstimateFundamentalMatrix8Point(normalized_points_a, normalized_points_b) * n_a;
}

}  // namespace nie

#endif  // NIE_CV_CALIB3D_FUNDAMENTAL_MATRIX_HPP
