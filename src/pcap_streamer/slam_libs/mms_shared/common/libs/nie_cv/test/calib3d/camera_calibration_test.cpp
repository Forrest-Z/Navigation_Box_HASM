/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <algorithm>

#include <gtest/gtest.h>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/gtest.hpp>
#include <nie/cv/calib3d/camera_calibration.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/extended_stereo_parameters.hpp>
#include <opencv2/calib3d.hpp>

class CameraCalibrationTest : public ::testing::Test {
protected:
    CameraCalibrationTest()
        : emp_(nie::io::ExtendedMonoParameters::Read(
                  "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/extended_mono.json")),
          esp_(nie::io::ExtendedStereoParameters::Read(
                  "/data/aiim/unit_tests_data/cv/calib3d/calibration/stereo/extended_stereo.json")) {}

    nie::io::ExtendedMonoParameters emp_;
    nie::io::ExtendedStereoParameters esp_;
};

constexpr double kEpsilon = 0.0025;
// In case of the intrinsics it is used as a percentage of change allowed.
constexpr double kEpsilonIntrinsics = 0.005;

// This is a "hack" to account for numeric instability of insignificant intrinsic parameters.
// They are usually the last few number in the list but we don't know which model is used.
// This is a practical ugly solution unless we want 32 different (amount of calibration
// possibilities) of these functions. It allows us to easily swap files.
void TestIntrinsics(std::vector<double> const& a, std::vector<double> const& b) {
    EXPECT_EQ(a.size(), b.size());

    std::size_t t = 7;
    std::size_t c = std::min(a.size(), t);

    for (std::size_t i = 0; i < c; ++i) {
        ASSERT_NEAR_RELATIVE(a[i], b[i], kEpsilonIntrinsics);
    }

    for (std::size_t i = t; i < a.size(); ++i) {
        ASSERT_NEAR_RELATIVE(a[i], b[i], 0.15);
    }
}

TEST_F(CameraCalibrationTest, DISABLED_Mono) {
    std::vector<std::vector<Eigen::Vector2f>> image_points = emp_.extended_data().image_points;
    std::vector<double> intrinsics;
    std::vector<nie::Isometry3qd> extrinsics;
    std::vector<std::vector<Eigen::Vector2f>> residuals;
    Eigen::MatrixXd cov_intrinsics;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics;
    double var_residuals;

    nie::DistortionModelParameters dmp{
            nie::ParameterFocalLengthToEnum(emp_.mono_parameters().distortion_parameters().focal_length),
            nie::ParameterSkewToEnum(emp_.mono_parameters().distortion_parameters().skew),
            nie::ParameterDistortionRadialToEnum(emp_.mono_parameters().distortion_parameters().distortion_radial),
            nie::ParameterDistortionTangentialToEnum(
                    emp_.mono_parameters().distortion_parameters().distortion_tangential),
            nie::ParameterDistortionThinPrismToEnum(
                    emp_.mono_parameters().distortion_parameters().distortion_thin_prism),
    };

    nie::EstimateCameraParametersCeres(
            dmp,
            emp_.object_points(),
            image_points,
            &intrinsics,
            &extrinsics,
            &residuals,
            &cov_intrinsics,
            &cov_extrinsics,
            &var_residuals);

    for (std::size_t i = 0; i < intrinsics.size(); ++i) {
        ASSERT_NEAR_RELATIVE(intrinsics[i], emp_.mono_parameters().lens().intrinsics[i], kEpsilon);
    }

    ASSERT_EQ(extrinsics.size(), emp_.extended_data().extrinsics.size());
    for (std::size_t i = 0; i < extrinsics.size(); ++i) {
        // translation
        ASSERT_NEAR(
                (extrinsics[i].translation() - emp_.extended_data().extrinsics[i].translation()).norm(), 0.0, kEpsilon);

        // rotation
        ASSERT_NEAR(
                (extrinsics[i].rotation().vec() - emp_.extended_data().extrinsics[i].rotation().vec()).norm(),
                0.0,
                kEpsilon);
    }

    // ...
    ASSERT_EQ(cov_intrinsics.size(), emp_.mono_parameters().lens().cov.size());
    for (int i = 0; i < cov_intrinsics.size(); ++i) {
        // changed from relative to this
        ASSERT_NEAR(cov_intrinsics.data()[i], emp_.mono_parameters().lens().cov.data()[i], kEpsilon);
    }
}

TEST_F(CameraCalibrationTest, DISABLED_Stereo) {
    std::vector<std::vector<Eigen::Vector2f>> image_points_left = esp_.extended_data_left().image_points;
    std::vector<std::vector<Eigen::Vector2f>> image_points_right = esp_.extended_data_right().image_points;
    std::vector<double> intrinsics_left;
    std::vector<double> intrinsics_right;
    std::vector<nie::Isometry3qd> extrinsics_left;
    std::vector<nie::Isometry3qd> extrinsics_right;
    nie::Isometry3qd baseline;
    std::vector<std::vector<Eigen::Vector2f>> residuals_left;
    std::vector<std::vector<Eigen::Vector2f>> residuals_right;
    Eigen::MatrixXd cov_intrinsics_left;
    Eigen::MatrixXd cov_intrinsics_right;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics_left;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics_right;
    Eigen::Matrix<double, 6, 6> cov_baseline;
    double var_residuals;

    nie::DistortionModelParameters dmp{
            nie::ParameterFocalLengthToEnum(esp_.distortion_parameters().focal_length),
            nie::ParameterSkewToEnum(esp_.distortion_parameters().skew),
            nie::ParameterDistortionRadialToEnum(esp_.distortion_parameters().distortion_radial),
            nie::ParameterDistortionTangentialToEnum(esp_.distortion_parameters().distortion_tangential),
            nie::ParameterDistortionThinPrismToEnum(esp_.distortion_parameters().distortion_thin_prism),
    };

    nie::EstimateStereoParametersCeres(
            dmp,
            esp_.object_points(),
            image_points_left,
            image_points_right,
            esp_.pair_count(),
            &intrinsics_left,
            &intrinsics_right,
            &extrinsics_left,
            &extrinsics_right,
            &baseline,
            &residuals_left,
            &residuals_right,
            &cov_intrinsics_left,
            &cov_intrinsics_right,
            &cov_extrinsics_left,
            &cov_extrinsics_right,
            &cov_baseline,
            &var_residuals);

    TestIntrinsics(intrinsics_left, esp_.stereo_parameters().lens_left().intrinsics);
    TestIntrinsics(intrinsics_right, esp_.stereo_parameters().lens_right().intrinsics);

    // Left
    ASSERT_EQ(extrinsics_left.size(), esp_.extended_data_left().extrinsics.size());
    for (std::size_t i = 0; i < extrinsics_left.size(); ++i) {
        // translation
        ASSERT_NEAR(
                (extrinsics_left[i].translation() - esp_.extended_data_left().extrinsics[i].translation()).norm(),
                0.0,
                kEpsilon);

        // rotation
        ASSERT_NEAR(
                (extrinsics_left[i].rotation().vec() - esp_.extended_data_left().extrinsics[i].rotation().vec()).norm(),
                0.0,
                kEpsilon);
    }

    // Right
    ASSERT_EQ(extrinsics_right.size(), esp_.extended_data_right().extrinsics.size());
    for (std::size_t i = 0; i < extrinsics_right.size(); ++i) {
        // translation
        ASSERT_NEAR(
                (extrinsics_right[i].translation() - esp_.extended_data_right().extrinsics[i].translation()).norm(),
                0.0,
                kEpsilon);

        // rotation
        ASSERT_NEAR(
                (extrinsics_right[i].rotation().vec() - esp_.extended_data_right().extrinsics[i].rotation().vec())
                        .norm(),
                0.0,
                kEpsilon);
    }

    // Baseline
    // translation
    ASSERT_NEAR(
            (baseline.translation() - esp_.stereo_parameters().baseline().estimate.translation()).norm(),
            0.0,
            kEpsilon);

    // rotation
    ASSERT_NEAR(
            (baseline.rotation().vec() - esp_.stereo_parameters().baseline().estimate.rotation().vec()).norm(),
            0.0,
            kEpsilon);

    // Covs
    ASSERT_EQ(cov_intrinsics_left.size(), emp_.mono_parameters().lens().cov.size());
    for (int i = 0; i < cov_intrinsics_left.size(); ++i) {
        // changed from relative to this
        ASSERT_NEAR(cov_intrinsics_left.data()[i], esp_.stereo_parameters().lens_left().cov.data()[i], 0.15);
    }

    ASSERT_EQ(cov_intrinsics_right.size(), emp_.mono_parameters().lens().cov.size());
    for (int i = 0; i < cov_intrinsics_right.size(); ++i) {
        // changed from relative to this
        ASSERT_NEAR(cov_intrinsics_right.data()[i], esp_.stereo_parameters().lens_right().cov.data()[i], 0.15);
    }
}
