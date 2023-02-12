/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/filesystem.hpp>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/formats/opencv.hpp>

TEST(LocalExtrinsicsTest, WriteAndRead) {
    constexpr double kPrecision = 1e-9;
    std::string tmp_filepath = nie::TemporaryFile("local_extrinsics_tmp.json").string();

    nie::Isometry3qd isometry_write;
    isometry_write.translation() << 3.6577316389204118e-01, 1.6141737407233550e-01, -9.3152609542673648e-01;
    isometry_write.rotation().coeffs() << -8.0890029752508114e-02, -2.9893566699628671e-01, -3.0233056035813170e-02,
        9.5035794962048836e-01;

    nie::io::OpenCvSerializer<nie::Isometry3qd>::Write(tmp_filepath, isometry_write);

    nie::Isometry3qd isometry_read = nie::io::OpenCvSerializer<nie::Isometry3qd>::Read(tmp_filepath);

    EXPECT_TRUE(isometry_read.isApprox(isometry_write, kPrecision));

    boost::filesystem::remove(tmp_filepath);
}
