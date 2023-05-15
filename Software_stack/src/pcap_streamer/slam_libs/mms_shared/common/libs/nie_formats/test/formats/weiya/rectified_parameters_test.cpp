/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/weiya/rectified_parameters.hpp>

#include "common.hpp"

// Sanity test by doing it twice through independent implementations.
TEST(CreateRectifiedParametersTest, FromWeiya) {
    auto calibration = nie::io::weiya::CameraCalibration::Read(kFilenameExtrinsicsXml);
    auto boresight = nie::io::weiya::CameraBoresight::Read(kFilenameAllBs);
    auto rectified = nie::io::weiya::RectifiedCameraParametersFromWeiya(kFilenameExtrinsicsXml, kFilenameAllBs);

    EXPECT_TRUE(calibration.K.isApprox(rectified.K));
    EXPECT_TRUE(GetCamToGnssLeft(calibration, boresight).isApprox(rectified.frames[0].baseline));
    EXPECT_TRUE(GetCamToGnssRight(calibration, boresight).isApprox(rectified.frames[1].baseline));
}