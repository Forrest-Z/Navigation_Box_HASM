/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_GEOMETRY_LINE_HPP
#define NIE_CV_GEOMETRY_LINE_HPP

#include <vector>

#include <Eigen/Dense>

namespace nie {

// Intersect two lines to find a 3D point. Since two lines won't generally
// intersect in 3D, we try to find the shortest distance between the two lines
// and take the center of this distance.
//
// Input:
//  Each line is defined by an origin and a direction.
// Output:
//  The 3D point and two scalars s0 and s1 that represent the scale factor by
//  which its corresponding vector can be scaled to find the point of minimum
//  distance along its corresponding line.
// Returns:
//  False in case or parallel lines or zero length lines.
template <typename T>
bool LineLineIntersect(
        // Line
        Eigen::Matrix<T, 3, 1> const& origin_0,
        Eigen::Matrix<T, 3, 1> const& vector_0,
        // Line
        Eigen::Matrix<T, 3, 1> const& origin_1,
        Eigen::Matrix<T, 3, 1> const& vector_1,
        // Out
        Eigen::Matrix<T, 3, 1>* x,
        T* s0,
        T* s1) {
    // Implicit casts to calculate with double accuracy.
    Eigen::Vector3d p0 = origin_0.template cast<double>();
    Eigen::Vector3d v0 = vector_0.template cast<double>();
    Eigen::Vector3d p1 = origin_1.template cast<double>();
    Eigen::Vector3d v1 = vector_1.template cast<double>();

    double dot_00 = v0.dot(v0);

    // Check if vector is not zero length.
    if (dot_00 < std::numeric_limits<double>::epsilon()) return false;

    double dot_11 = v1.dot(v1);

    // Check if vector is not zero length.
    if (dot_11 < std::numeric_limits<double>::epsilon()) return false;

    Eigen::Vector3d vp = p0 - p1;

    double dot_p1 = vp.dot(v1);
    double dot_10 = v1.dot(v0);
    double dot_p0 = vp.dot(v0);

    double denominator = dot_00 * dot_11 - dot_10 * dot_10;

    // Check for division by (close to) zero, meaning parallel lines
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) return false;

    double numerator = dot_p1 * dot_10 - dot_p0 * dot_11;

    double s0d = numerator / denominator;
    double s1d = (dot_p1 + dot_10 * s0d) / dot_11;

    Eigen::Vector3d p10 = p0 + v0 * s0d;
    Eigen::Vector3d p32 = p1 + v1 * s1d;
    Eigen::Vector3d xd = (p10 + p32) * 0.5;

    *x = xd.cast<T>();
    *s0 = static_cast<T>(s0d);
    *s1 = static_cast<T>(s1d);

    return true;
}

// Intersect two or more lines to find a 3D point. Since two (or more) lines won't
// generally intersect in 3D, we try to minimize the sum of distances between the
// 3d point and all lines. The minimization problem is defined as:
//
//  sum || X - ((X - O) dot V V + O) ||^2
//
// Input:
//  Each line is defined by an origin and a direction.
// Output:
//  The 3D point.
//
// Note:
//  "Normalization" is now handled by subtracting the first origin and then adding that to the result.
//  TODO(jbr): Improve numeric stability / matrix conditioning - Perhaps normalize over all origins.
template <typename Origins, typename Vectors>
bool LinesIntersect(Origins const& origins, Vectors const& vectors, Eigen::Vector3d* x) {
    assert(origins.size() > 1);
    assert(origins.size() == vectors.size());

    // Find the solution to A^tAx=A^tb instead of Ax=b for fixed size matrices.
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();  // A is actually A^tA
    Eigen::Vector3d b = Eigen::Vector3d::Zero();  // b is actually A^tb

    for (std::size_t i = 0; i < origins.size(); ++i) {
        Eigen::Vector3d origin_i = origins[i] - origins[0];
        Eigen::Vector3d vector_i = vectors[i].template normalized();

        Eigen::Matrix3d rows_A_i = Eigen::Matrix3d::Identity() - vector_i * vector_i.transpose();
        Eigen::Vector3d b_i = -origin_i.dot(vector_i) * vector_i + origin_i;

        // Since rows_A_i is symmetrical we don't need to transpose it for A^tA and A^tb
        A += rows_A_i * rows_A_i;
        b += rows_A_i * b_i;
    }

    Eigen::ColPivHouseholderQR<Eigen::Matrix3d> decomposition(A);
    // If the result isn't valid it means that A is not invertible which happens
    // when all lines are parallel or there aren't a minimum of 2 well defined
    // lines.
    *x = origins[0] + decomposition.solve(b);
    // Assuming that for a rank revealing decomposition it is cheap to check if it's invertible.
    return decomposition.isInvertible();
}

}  // namespace nie

#endif  // NIE_CV_GEOMETRY_LINE_HPP
