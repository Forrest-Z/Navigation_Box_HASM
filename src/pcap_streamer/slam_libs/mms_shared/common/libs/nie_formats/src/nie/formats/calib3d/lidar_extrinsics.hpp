/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LAS_CREATOR_SRC_READ_LIDAR_EXTRINSICS_HPP_
#define LAS_CREATOR_SRC_READ_LIDAR_EXTRINSICS_HPP_

#include <string>

#include <glog/logging.h>

#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/geometry/rotation.hpp>

namespace nie {

/// NOTE: We know from reverse engineering, getting the exact same data as China.

/// Create the calibration transformation that transforms points in the sensor frame to points in the reference frame.
///
/// The reference frame is defined to be at the position of the IMU, with the axes the same as the vehicle.
///
/// The axes direction of the vehicle are defined from the inside of the vehicle:
///      +X is towards the right side
///      +Y is towards the front
///      +Z is towards the roof
///
/// Calibration parameters (x,y,z) in meters and (r,p,h) in degrees, as defined in the NavInfo Calibration Database
/// NOTE: Translation and rotation in the NavInfo Calibration Database are defined in an inverted sense.
///
/// \param x
/// \param y
/// \param z
/// \param r roll [degrees]
/// \param p pitch [degrees]
/// \param h heading [degrees]
/// \return Isometry3qd
template <typename Scalar>
inline Isometry3q<Scalar> CalibrationTransform(
    Scalar const x, Scalar const y, Scalar const z, Scalar const r, Scalar const p, Scalar const h) {
    return Isometry3q<Scalar>{{x, y, z},
                              EulerToQuaternion<Axis::Z, Axis::Y, Axis::X>(Deg2Rad(-h), Deg2Rad(-r), Deg2Rad(-p))};
}

/// For now a temporary format for this data:
///     MDL
///     -43.671 0.503359 90.6875          roll pitch heading
///     0.0202054 -0.325774 0.213096      x y z
///
/// \param filepath
/// \return
Isometry3qd ReadLidarExtrinsics(std::string const& filepath);

}  // namespace nie

#endif  // LAS_CREATOR_SRC_READ_LIDAR_EXTRINSICS_HPP_
