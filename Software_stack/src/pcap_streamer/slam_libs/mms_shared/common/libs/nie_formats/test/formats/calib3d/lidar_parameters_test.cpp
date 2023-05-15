/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/filesystem.hpp>
#include <nie/formats/calib3d/lidar_parameters.hpp>

class LidarParametersTest : public ::testing::Test {
protected:
    // clang-format off
    nie::io::LidarParameters const kMono{
        nie::io::LidarType::kVelodyneVLP16,
        "velodyne_Rear",
        "192.168.1.201",
        2368,
        0.0,
        357.0,
        nie::Isometry3qd::Identity()
        };
    // clang-format on

    std::string const kInputDir{"/data/aiim/unit_tests_data/cv/calib3d/calibration/"};
};

TEST_F(LidarParametersTest, VerifyUnknown) {
    std::string nonsense = "i_dont_want_to_be_a_valid_lidar_string";
    EXPECT_DEATH(nie::io::LidarTypeStringToEnum(nonsense), "Unknown LiDAR type: " + nonsense);
}

// Test the string conversion to enum and their respective order which should be identical.
TEST_F(LidarParametersTest, LidarTypeStringToEnum) {
    for (std::size_t index = 0; index < nie::io::kLidarTypeStrings.size(); ++index) {
        std::string type_string_in = nie::io::kLidarTypeStrings[index];
        nie::io::LidarType type_enum = nie::io::LidarTypeStringToEnum(type_string_in);
        EXPECT_EQ(type_enum, static_cast<nie::io::LidarType>(index));
    }
}

TEST_F(LidarParametersTest, MonoRead) {
    auto const read = nie::io::LidarParameters::Read(kInputDir + "mono_lidar_parameters_with_port_validated.json");
    EXPECT_EQ(kMono, read);
}

TEST_F(LidarParametersTest, MonoWrite) {
    auto filename = nie::TemporaryFile("mono_lidar_parameters_with_port.json").string();
    kMono.Write(filename);
    EXPECT_TRUE(nie::BinaryEquals(filename, kInputDir + "mono_lidar_parameters_with_port_validated.json"));
}