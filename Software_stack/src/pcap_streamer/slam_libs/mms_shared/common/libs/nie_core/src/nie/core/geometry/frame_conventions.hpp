/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "isometry3.hpp"
#include "nie/core/constants.hpp"
#include "rotation.hpp"

/// @file frame_conventions.hpp
/// @brief Provides some default non customer specific support for common conventions.
/// @details The frames described in this file are considered common. They are described below:

namespace nie {

enum class Frame {
    /// Aircraft frame:
    /// x (Right)
    /// y (Forward)
    /// z (Up)
    /// Euler angles based matrix as: y (roll) * x (pitch) * z (yaw)
    /// GIS packages such as Inertial Explorer can express the yaw has -heading.
    /// https://en.wikipedia.org/wiki/Aircraft_principal_axes
    ///
    /// Note: z is also often the (Down) axis. This is left handed and not supported by quaternions.
    ///
    kAircraft,
    /// Vehicle frame:
    /// x (Forward)
    /// y (Left)
    /// z (Up)
    /// Euler angles based matrix as: x (roll) * y (pitch) * z (yaw)
    /// See: https://www.sis.se/api/document/preview/914200/
    kVehicle,
    /// CV frame:
    /// x (Right)
    /// y (Down)
    /// z (Forward)
    /// Euler angles are never imported or exported. Only quaternions or matrices are used. No convention required.
    /// https://docs.opencv.org/3.4.1/d9/d0c/group__calib3d.html
    kCv
};

namespace detail {

template <Frame From, Frame To, typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3> kRotationFromToMatrix = {};

// clang-format off
// Aircraft to Cv = 90 degrees about X
template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kAircraft, Frame::kCv, Scalar> = (
    Eigen::Matrix<Scalar, 3, 3>{} <<
         1.0,  0.0,  0.0,   // Cv x = Aircraft x
         0.0,  0.0, -1.0,   // Cv y = Aircraft -z
         0.0,  1.0,  0.0    // Cv z = Aircraft y
).finished();

template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kCv, Frame::kAircraft, Scalar> =
    kRotationFromToMatrix<Frame::kAircraft, Frame::kCv, Scalar>.transpose();

// Aircraft to vehicle = -90 degrees about Z
template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kAircraft, Frame::kVehicle, Scalar> = (
    Eigen::Matrix<Scalar, 3, 3>{} <<
         0.0,  1.0,  0.0,   // Vehicle x = Aircraft y
        -1.0,  0.0,  0.0,   // Vehicle y = Aircraft -x
         0.0,  0.0,  1.0    // Vehicle z = Aircraft z
).finished();

template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kVehicle, Frame::kAircraft, Scalar> =
    kRotationFromToMatrix<Frame::kAircraft, Frame::kVehicle, Scalar>.transpose();

// Vehicle to Cv
template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kVehicle, Frame::kCv, Scalar> = (
    Eigen::Matrix<Scalar, 3, 3>{} <<
         0.0, -1.0,  0.0,   // Cv x = Vehicle -y
         0.0,  0.0, -1.0,   // Cv y = Vehicle -z
         1.0,  0.0,  0.0    // Cv z = Vehicle x
).finished();

template<typename Scalar>
static const Eigen::Matrix<Scalar, 3, 3>
kRotationFromToMatrix<Frame::kCv, Frame::kVehicle, Scalar> =
    kRotationFromToMatrix<Frame::kVehicle, Frame::kCv, Scalar>.transpose();

// clang-format on

template <Frame From, Frame To, typename Scalar>
static const Eigen::Quaternion<Scalar> kRotationFromToQuaternion{kRotationFromToMatrix<From, To, Scalar>};

template <
        Frame From,
        Frame To,
        typename Derived,
        typename Scalar = typename Eigen::MatrixBase<Derived>::Scalar,
        typename std::enable_if_t<Derived::ColsAtCompileTime == 1, int> = 0>
Eigen::Matrix<Scalar, 3, 1> ConvertFrameVector(Eigen::MatrixBase<Derived> const& point) {
    static_assert(Derived::RowsAtCompileTime == 3, "Function input not 3x1.");
    return kRotationFromToMatrix<From, To, Scalar> * point;
}

template <
        Frame From,
        Frame To,
        typename Derived,
        typename Scalar = typename Eigen::MatrixBase<Derived>::Scalar,
        typename std::enable_if_t<Derived::ColsAtCompileTime == 3, int> = 0>
Eigen::Matrix<Scalar, 3, 3> ConvertFrameRotation(Eigen::MatrixBase<Derived> const& m) {
    static_assert(Derived::RowsAtCompileTime == 3, "Function input not 3x3.");
    return kRotationFromToMatrix<From, To, Scalar> * m * kRotationFromToMatrix<To, From, Scalar>;
}

template <Frame From, Frame To, typename Derived, typename Scalar = typename Eigen::QuaternionBase<Derived>::Scalar>
Eigen::Quaternion<Scalar> ConvertFrameRotation(Eigen::QuaternionBase<Derived> const& q) {
    return kRotationFromToQuaternion<From, To, Scalar> * q * kRotationFromToQuaternion<To, From, Scalar>;
}

}  // namespace detail

template <Frame From, Frame To, typename Derived, typename Rotation>
nie::Isometry3<Rotation> ConvertFrame(nie::Isometry3Base<Derived, Rotation> const& v) {
    using namespace detail;
    return {ConvertFrameVector<From, To>(v.translation()), ConvertFrameRotation<From, To>(v.rotation())};
}

template <Frame From, Frame To, typename Rotation, typename Scalar = typename Rotation::Scalar>
static nie::Isometry3<Rotation> const kIsometryFromTo = {};

template <Frame From, Frame To, typename Scalar>
static nie::Isometry3m<Scalar> const kIsometryFromTo<From, To, Eigen::Matrix<Scalar, 3, 3>, Scalar> = {
        Eigen::Matrix<Scalar, 3, 1>::Identity(), detail::kRotationFromToMatrix<From, To, Scalar>};

template <Frame From, Frame To, typename Scalar>
static nie::Isometry3q<Scalar> const kIsometryFromTo<From, To, Eigen::Quaternion<Scalar>, Scalar> = {
        Eigen::Matrix<Scalar, 3, 1>::Identity(), detail::kRotationFromToQuaternion<From, To, Scalar>};

}  // namespace nie
