/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/weiya/camera_boresight.hpp>

#include "common.hpp"

// Camera to IMU:
// /data/aiim/unit_tests_data/formats/weiya/all.bs

// clang-format off
const nie::Isometry3qd kCameraToImu{
    Eigen::Vector3d{-9.6409868121003828e-01, 2.0423156993044950e-02, 1.5814381553813728e-01},
    Eigen::Quaterniond{(Eigen::Matrix3d{} <<
         9.9999742396875002e-01,    -4.8727670291072385e-04,    -2.2168936101736010e-03,
         2.2166714579254553e-03,    -4.5597397685022737e-04,     9.9999743922441120e-01,
        -4.8828630090012274e-04,    -9.9999977732454881e-01,    -4.5489266988755963e-04
    ).finished()}};
// clang-format on

TEST(CameraBoresightTest, ReadAllBs) {
    nie::Isometry3qd cam_to_imu = nie::io::weiya::CameraBoresight::Read(kFilenameAllBs);

    EXPECT_TRUE(cam_to_imu.isApprox(kCameraToImu));
}