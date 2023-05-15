/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/weiya/camera_calibration.hpp>

#include "common.hpp"

// /data/aiim/Data_Coremap/Project1001-0003-190403-03/Output/extrinsics.xml
// R1
// clang-format off
const Eigen::Matrix3d kR1 = (
    Eigen::Matrix3d{} <<
         9.9998399161979090e-01,     2.4097124507090366e-03,    5.1195497902698098e-03,
        -2.4225241321563042e-03,     9.9999394622761073e-01,    2.4977760026657744e-03,
        -5.1134998757479243e-03,    -2.5101382502305549e-03,    9.9998377553087592e-01
).finished();

const Eigen::Vector3d kT = (
    Eigen::Vector3d{} <<
        -9.9996338633176242e+02,     1.0283770655154729e+00,    -3.7114102220566338e+00
).finished() / 1000.0;

// P1.block<3, 3>(0, 0)
const Eigen::Matrix3d kKP1 = (
    Eigen::Matrix3d{} <<
        2.4166297778024405e+03, 0.,                     2.0441625709533691e+03,
        0.,                     2.4166297778024405e+03, 1.0836907596588135e+03,
        0.,                     0.,                     1.
).finished();

// clang-format on

TEST(CameraCalibrationTest, ReadExtrinsicsXml) {
    nie::io::weiya::CameraCalibration extrinsics_xml = nie::io::weiya::CameraCalibration::Read(kFilenameExtrinsicsXml);
    nie::Isometry3qd left = nie::Isometry3qd::FromRotation(Eigen::Quaterniond{kR1.transpose()});
    nie::Isometry3qd right = nie::Isometry3qd::FromTranslation(Eigen::Vector3d{kT.norm(), 0.0, 0.0});

    EXPECT_TRUE(kKP1.isApprox(extrinsics_xml.K));
    EXPECT_TRUE(left.isApprox(extrinsics_xml.T_gl));
    EXPECT_TRUE(right.isApprox(extrinsics_xml.T_lr));
}