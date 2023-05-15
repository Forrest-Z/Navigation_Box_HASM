/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <nie/core/filesystem.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/extended_stereo_parameters.hpp>

class ExtendedCalibrationDataTest : public testing::Test {};

TEST_F(ExtendedCalibrationDataTest, Mono) {
    nie::io::ExtendedMonoParameters mp = nie::io::ExtendedMonoParameters::Read(
        "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/extended_mono.json");

    std::string temp_filename = nie::TemporaryFile("mock_output_mono.json").string();

    nie::io::ExtendedMonoParameters::Write(
        temp_filename,
        mp.image_size(),
        mp.distortion_parameters(),
        mp.pattern_size(),
        mp.square_size(),
        mp.object_points(),
        mp.var_residuals(),
        mp.mono_parameters().lens().id,
        mp.extended_data().image_ids,
        mp.extended_data().image_points,
        mp.mono_parameters().lens().intrinsics,
        mp.extended_data().extrinsics,
        mp.extended_data().residuals,
        mp.mono_parameters().lens().cov,
        mp.extended_data().cov_extrinsics);

    nie::io::ExtendedMonoParameters test_read = nie::io::ExtendedMonoParameters::Read(temp_filename);

    EXPECT_TRUE(test_read.distortion_parameters().distortion_radial == mp.distortion_parameters().distortion_radial);
    EXPECT_TRUE(
        test_read.distortion_parameters().distortion_tangential == mp.distortion_parameters().distortion_tangential);
    EXPECT_TRUE(
        test_read.distortion_parameters().distortion_thin_prism == mp.distortion_parameters().distortion_thin_prism);
    EXPECT_TRUE(test_read.distortion_parameters().focal_length == mp.distortion_parameters().focal_length);
    EXPECT_TRUE(test_read.distortion_parameters().skew == mp.distortion_parameters().skew);

    EXPECT_TRUE(test_read.image_size() == mp.image_size());
    EXPECT_TRUE(test_read.pattern_size() == mp.pattern_size());
    EXPECT_TRUE(test_read.square_size() == mp.square_size());
    EXPECT_TRUE(test_read.object_points() == mp.object_points());
    EXPECT_TRUE(test_read.var_residuals() == mp.var_residuals());

    EXPECT_TRUE(test_read.mono_parameters().lens().id == mp.mono_parameters().lens().id);
    EXPECT_TRUE(test_read.mono_parameters().lens().intrinsics == mp.mono_parameters().lens().intrinsics);
    EXPECT_TRUE(test_read.mono_parameters().lens().cov == mp.mono_parameters().lens().cov);

    EXPECT_TRUE(test_read.extended_data().image_ids == mp.extended_data().image_ids);
    EXPECT_TRUE(test_read.extended_data().image_points == mp.extended_data().image_points);
    EXPECT_TRUE(std::equal(
        test_read.extended_data().extrinsics.cbegin(),
        test_read.extended_data().extrinsics.cend(),
        mp.extended_data().extrinsics.cbegin(),
        mp.extended_data().extrinsics.cend(),
        [](nie::Isometry3qd const& lhs, nie::Isometry3qd const& rhs) {
            return lhs.translation().isApprox(rhs.translation()) && lhs.rotation().isApprox(rhs.rotation());
        }));
    EXPECT_TRUE(test_read.extended_data().residuals == mp.extended_data().residuals);
    EXPECT_TRUE(test_read.extended_data().cov_extrinsics == mp.extended_data().cov_extrinsics);
}

TEST_F(ExtendedCalibrationDataTest, Stereo) {
    nie::io::ExtendedStereoParameters sp = nie::io::ExtendedStereoParameters::Read(
        "/data/aiim/unit_tests_data/cv/calib3d/calibration/stereo/extended_stereo.json");

    std::string temp_filename = nie::TemporaryFile("mock_output_stereo.json").string();

    nie::io::ExtendedStereoParameters::Write(
        temp_filename,
        sp.image_size(),
        sp.distortion_parameters(),
        sp.pattern_size(),
        sp.square_size(),
        sp.object_points(),
        sp.var_residuals(),
        sp.stereo_parameters().lens_left().id,
        sp.stereo_parameters().lens_right().id,
        sp.extended_data_left().image_ids,
        sp.extended_data_right().image_ids,
        sp.extended_data_left().image_points,
        sp.extended_data_right().image_points,
        sp.stereo_parameters().lens_left().intrinsics,
        sp.stereo_parameters().lens_right().intrinsics,
        sp.extended_data_left().extrinsics,
        sp.extended_data_right().extrinsics,
        sp.extended_data_left().residuals,
        sp.extended_data_right().residuals,
        sp.stereo_parameters().lens_left().cov,
        sp.stereo_parameters().lens_right().cov,
        sp.extended_data_left().cov_extrinsics,
        sp.extended_data_right().cov_extrinsics,
        sp.pair_count(),
        sp.stereo_parameters().baseline().estimate,
        sp.stereo_parameters().baseline().cov);

    nie::io::ExtendedStereoParameters test_read = nie::io::ExtendedStereoParameters::Read(temp_filename);

    EXPECT_TRUE(test_read.distortion_parameters().distortion_radial == sp.distortion_parameters().distortion_radial);
    EXPECT_TRUE(
        test_read.distortion_parameters().distortion_tangential == sp.distortion_parameters().distortion_tangential);
    EXPECT_TRUE(
        test_read.distortion_parameters().distortion_thin_prism == sp.distortion_parameters().distortion_thin_prism);
    EXPECT_TRUE(test_read.distortion_parameters().focal_length == sp.distortion_parameters().focal_length);
    EXPECT_TRUE(test_read.distortion_parameters().skew == sp.distortion_parameters().skew);

    EXPECT_TRUE(test_read.image_size() == sp.image_size());
    EXPECT_TRUE(test_read.pattern_size() == sp.pattern_size());
    EXPECT_TRUE(test_read.square_size() == sp.square_size());
    EXPECT_TRUE(test_read.object_points() == sp.object_points());
    EXPECT_TRUE(test_read.var_residuals() == sp.var_residuals());
    EXPECT_TRUE(test_read.pair_count() == sp.pair_count());

    EXPECT_TRUE(test_read.stereo_parameters().lens_left().id == sp.stereo_parameters().lens_left().id);
    EXPECT_TRUE(test_read.stereo_parameters().lens_right().id == sp.stereo_parameters().lens_right().id);
    EXPECT_TRUE(
        test_read.stereo_parameters().baseline().estimate.translation() ==
        sp.stereo_parameters().baseline().estimate.translation());
    EXPECT_TRUE(test_read.stereo_parameters().baseline().estimate.rotation().isApprox(
        sp.stereo_parameters().baseline().estimate.rotation()));
    EXPECT_TRUE(test_read.stereo_parameters().baseline().cov == sp.stereo_parameters().baseline().cov);
    EXPECT_TRUE(test_read.stereo_parameters().lens_left().intrinsics == sp.stereo_parameters().lens_left().intrinsics);
    EXPECT_TRUE(
        test_read.stereo_parameters().lens_right().intrinsics == sp.stereo_parameters().lens_right().intrinsics);
    EXPECT_TRUE(test_read.stereo_parameters().lens_left().cov == sp.stereo_parameters().lens_left().cov);
    EXPECT_TRUE(test_read.stereo_parameters().lens_right().cov == sp.stereo_parameters().lens_right().cov);

    EXPECT_TRUE(test_read.extended_data_left().image_ids == sp.extended_data_left().image_ids);
    EXPECT_TRUE(test_read.extended_data_left().image_points == sp.extended_data_left().image_points);
    EXPECT_TRUE(std::equal(
        test_read.extended_data_left().extrinsics.cbegin(),
        test_read.extended_data_left().extrinsics.cend(),
        sp.extended_data_left().extrinsics.cbegin(),
        sp.extended_data_left().extrinsics.cend(),
        [](nie::Isometry3qd const& lhs, nie::Isometry3qd const& rhs) {
            return lhs.translation().isApprox(rhs.translation()) && lhs.rotation().isApprox(rhs.rotation());
        }));
    EXPECT_TRUE(test_read.extended_data_left().residuals == sp.extended_data_left().residuals);
    EXPECT_TRUE(test_read.extended_data_left().cov_extrinsics == sp.extended_data_left().cov_extrinsics);

    EXPECT_TRUE(test_read.extended_data_right().image_ids == sp.extended_data_right().image_ids);
    EXPECT_TRUE(test_read.extended_data_right().image_points == sp.extended_data_right().image_points);
    EXPECT_TRUE(std::equal(
        test_read.extended_data_right().extrinsics.cbegin(),
        test_read.extended_data_right().extrinsics.cend(),
        sp.extended_data_right().extrinsics.cbegin(),
        sp.extended_data_right().extrinsics.cend(),
        [](nie::Isometry3qd const& lhs, nie::Isometry3qd const& rhs) {
            return lhs.translation().isApprox(rhs.translation()) && lhs.rotation().isApprox(rhs.rotation());
        }));
    EXPECT_TRUE(test_read.extended_data_right().residuals == sp.extended_data_right().residuals);
    EXPECT_TRUE(test_read.extended_data_right().cov_extrinsics == sp.extended_data_right().cov_extrinsics);
}
