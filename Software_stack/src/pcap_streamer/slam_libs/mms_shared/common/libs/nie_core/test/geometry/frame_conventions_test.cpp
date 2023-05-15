/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/core/geometry/frame_conventions.hpp>

TEST(FameConventionsTest, VehicleToCv) {
    double const kEasting = 1.0;
    double const kNorthing = 2.0;
    double const kHeight = 3.0;

    // Isometry is in CV frame:
    // x right    (negative northing)
    // y down     (negative height)
    // z forward  (positive easting)

    Eigen::Matrix<double, 3, 1> expected{-kNorthing, -kHeight, kEasting};
    Eigen::Matrix<double, 3, 1> translation = nie::detail::ConvertFrameVector<nie::Frame::kVehicle, nie::Frame::kCv>(
        Eigen::Matrix<double, 3, 1>{kEasting, kNorthing, kHeight});

    ASSERT_EQ(translation, expected);
}