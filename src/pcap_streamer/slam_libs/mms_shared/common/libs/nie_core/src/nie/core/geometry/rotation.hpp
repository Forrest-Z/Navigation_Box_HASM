/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Geometry>

#include "nie/core/constants.hpp"

namespace nie {

namespace detail {

template <typename Scalar>
inline static Eigen::Matrix<Scalar, 3, 1> AngleAxisVector3(Eigen::Matrix<Scalar, 3, 1> const& unit_axis, Scalar angle) {
    return unit_axis * angle;
}

template <typename Scalar>
inline static Eigen::AngleAxis<Scalar> AngleAxis(Eigen::Matrix<Scalar, 3, 1> const& unit_axis, Scalar angle) {
    return {angle, unit_axis};
}

template <typename Scalar>
inline static Eigen::Quaternion<Scalar> Quaternion(Eigen::Matrix<Scalar, 3, 1> const& unit_axis, Scalar angle) {
    return Eigen::Quaternion<Scalar>{AngleAxis(unit_axis, angle)};
}

template <typename Scalar>
inline static Eigen::Matrix<Scalar, 3, 3> Matrix(Eigen::Matrix<Scalar, 3, 1> const& unit_axis, Scalar angle) {
    return Eigen::Matrix<Scalar, 3, 3>{AngleAxis(unit_axis, angle)};
}

}  // namespace detail

template <typename T = double>
constexpr T Deg2Rad(T degrees) {
    return degrees * kDeg2Rad<T>;
}

template <typename T = double>
constexpr T Rad2Deg(T radians) {
    return radians * kRad2Deg<T>;
}

enum class Axis : size_t { X = 0, Y, Z };

// Nonsense (primary example like an empty struct).
template <Axis A, typename Scalar>
static const Eigen::Matrix<Scalar, 3, 1> kUnitAxis{};

template <typename Scalar>
static const Eigen::Matrix<Scalar, 3, 1> kUnitAxis<Axis::X, Scalar> = Eigen::Matrix<Scalar, 3, 1>::UnitX();

template <typename Scalar>
static const Eigen::Matrix<Scalar, 3, 1> kUnitAxis<Axis::Y, Scalar> = Eigen::Matrix<Scalar, 3, 1>::UnitY();

template <typename Scalar>
static const Eigen::Matrix<Scalar, 3, 1> kUnitAxis<Axis::Z, Scalar> = Eigen::Matrix<Scalar, 3, 1>::UnitZ();

template <Axis A, typename Scalar>
inline Eigen::AngleAxis<Scalar> MakeAngleAxis(Scalar angle) {
    return detail::AngleAxis(kUnitAxis<A, Scalar>, angle);
}

template <Axis A, typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> MakeAngleAxisVector3(Scalar angle) {
    return detail::AngleAxisVector3(kUnitAxis<A, Scalar>, angle);
}

template <Axis A, typename Scalar>
inline Eigen::Quaternion<Scalar> MakeQuaternion(Scalar angle) {
    return detail::Quaternion(kUnitAxis<A, Scalar>, angle);
}

template <Axis A, typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> MakeMatrix(Scalar angle) {
    return detail::Matrix(kUnitAxis<A, Scalar>, angle);
}

/// Convert Euler angles to Quaternion using XYZ order of rotations.
/// First the yaw rotation is applied (z-axis), then the pitch rotation (y-axis) and finally the roll (x-axis).
/// Matrix: R_xyz = x * y * z.
///
/// Default values follow the ISO 8855 standard for vehicles:
/// reference: https://www.mathworks.com/help/driving/ug/coordinate-systems.html
/// x forward
/// y left
/// z up
/// roll    positive roll is left-hand wheels lifted from the ground (clockwise angle around x-axis)
/// pitch   positive pitch is down hill (clockwise around y-axis)
/// yaw     positive yaw is left-hand turn of vehicle (clockwise around z-axis)
///
/// \tparam T
/// \tparam Axis0  default x-axis
/// \tparam Axis1  default y-axis
/// \tparam Axis2  default z-axis
/// \param angle0  default roll     (clockwise around Axis0)
/// \param angle1  default pitch    (clockwise around Axis1)
/// \param angle2  default yaw      (clockwise around Axis2)
/// \return  normalized quaternion.
template <Axis Axis0 = Axis::X, Axis Axis1 = Axis::Y, Axis Axis2 = Axis::Z, typename T>
inline Eigen::Quaternion<T> EulerToQuaternion(T const& angle0, T const& angle1, T const& angle2) {
    Eigen::Quaternion<T> q(MakeAngleAxis<Axis0>(angle0) * MakeAngleAxis<Axis1>(angle1) * MakeAngleAxis<Axis2>(angle2));
    q.normalize();
    return q;
}

template <Axis Axis0 = Axis::X, Axis Axis1 = Axis::Y, Axis Axis2 = Axis::Z, typename T>
inline Eigen::Matrix<T, 3, 3> EulerToMatrix(T const& angle0, T const& angle1, T const& angle2) {
    return MakeMatrix<Axis0>(angle0) * MakeMatrix<Axis1>(angle1) * MakeMatrix<Axis2>(angle2);
}

}  // namespace nie
