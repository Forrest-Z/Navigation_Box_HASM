/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/filesystem.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>

class RectifiedParametersTest : public ::testing::Test {
protected:
    // clang-format off
    nie::io::RectifiedCameraParameters const kMono{
        "second",
        Eigen::Vector2i{1920, 1200},
        (Eigen::Matrix3d() <<
            1.3134678981095021e+03,                     0.0, 9.7914838005798140e+02,
                               0.0,  1.3134678981095021e+03, 6.0529607417856732e+02,
                               0.0,                     0.0,                    1.0).finished(),
        {{"left", nie::Isometry3qd::Identity()}}};

    nie::io::RectifiedCameraParameters const kStereo{
        "second",
        Eigen::Vector2i{1920, 1200},
        (Eigen::Matrix3d() <<
        1.3424589067576360e+03,                    0.0,  9.7335200504558952e+02,
                           0.0, 1.3424589067576360e+03,  6.1147721615060175e+02,
                           0.0,                    0.0,                     1.0).finished(),
        {
            {"left", nie::Isometry3qd::Identity()},
            {"right", nie::Isometry3qd::FromTranslation(
                Eigen::Vector3d{4.0995810924262344e-01, 0.0, -5.4210108624275222e-20})
            }
        }
    };
    // clang-format on

    std::string const kOutputDir{"/data/aiim/unit_tests_data/cv/calib3d/calibration/"};
};

TEST_F(RectifiedParametersTest, MonoRead) {
    auto const read =
        nie::io::RectifiedCameraParameters::Read(kOutputDir + "mono_vector_rectified_intrinsics_validated.json");
    EXPECT_EQ(kMono, read);
}

TEST_F(RectifiedParametersTest, MonoWrite) {
    auto filename = nie::TemporaryFile("mono_rectified_intrinsics.json").string();
    kMono.Write(filename);
    EXPECT_TRUE(nie::BinaryEquals(filename, kOutputDir + "mono_vector_rectified_intrinsics_validated.json"));
}

TEST_F(RectifiedParametersTest, StereoRead) {
    auto const read =
        nie::io::RectifiedCameraParameters::Read(kOutputDir + "stereo_vector_rectified_intrinsics_validated.json");
    EXPECT_EQ(kStereo, read);
}

TEST_F(RectifiedParametersTest, StereoWrite) {
    auto filename = nie::TemporaryFile("stereo_rectified_intrinsics.json").string();
    kStereo.Write(filename);
    EXPECT_TRUE(nie::BinaryEquals(filename, kOutputDir + "stereo_vector_rectified_intrinsics_validated.json"));
}
