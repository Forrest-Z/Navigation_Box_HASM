/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/calib3d/lidar_extrinsics.hpp>

namespace {

// Create the calibration transformation that transforms points in the sensor frame to points in the reference frame.
//
// The reference frame is defined to be at the position of the IMU, with the axes the same as the vehicle.
//
// The axes direction of the vehicle are defined from the inside of the vehicle:
//      +X is towards the right side
//      +Y is towards the front
//      +Z is towards the roof
//
// Calibration parameters (x,y,z) in meters and (r,p,h) in degrees, as defined in the NavInfo Calibration Database
// NOTE: Translation and rotation in the NavInfo Calibration Database are defined in an inverted sense.
Eigen::Isometry3d CalibrationTransformOld(double x, double y, double z, double r, double p, double h) {
    auto T = Eigen::Isometry3d::Identity();

    T.translate(Eigen::Vector3d(x, y, z));
    T.rotate(Eigen::AngleAxisd(nie::Deg2Rad(-h), Eigen::Vector3d::UnitZ()));  // Heading rotates Z axis
    T.rotate(Eigen::AngleAxisd(nie::Deg2Rad(-r), Eigen::Vector3d::UnitY()));  // Roll    rotates Y axis
    T.rotate(Eigen::AngleAxisd(nie::Deg2Rad(-p), Eigen::Vector3d::UnitX()));  // Pitch   rotates X axis

    return T;
}
}  // namespace

TEST(CalibrationTransform, CalibrationTransformOld) {
    constexpr double kPrecision = 1e-12;

    // Some values that are random enough
    double const x = 3.456;
    double const y = 0.4;
    double const z = -2.567;
    double const r = 0.876;
    double const p = -0.567;
    double const h = -1.234;

    nie::Isometry3qd isometry = nie::CalibrationTransform(x, y, z, r, p, h);
    Eigen::Isometry3d isometry_old_eigen = CalibrationTransformOld(x, y, z, r, p, h);

    nie::Isometry3qd isometry_old{isometry_old_eigen.translation(), Eigen::Quaterniond{isometry_old_eigen.rotation()}};

    EXPECT_NEAR(isometry.translation().x(), isometry_old.translation().x(), kPrecision);
    EXPECT_NEAR(isometry.translation().y(), isometry_old.translation().y(), kPrecision);
    EXPECT_NEAR(isometry.translation().z(), isometry_old.translation().z(), kPrecision);
    EXPECT_NEAR(isometry.rotation().x(), isometry_old.rotation().x(), kPrecision);
    EXPECT_NEAR(isometry.rotation().y(), isometry_old.rotation().y(), kPrecision);
    EXPECT_NEAR(isometry.rotation().z(), isometry_old.rotation().z(), kPrecision);
    EXPECT_NEAR(isometry.rotation().w(), isometry_old.rotation().w(), kPrecision);
}
