/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef TEST_FORMATS_WEIYA
#define TEST_FORMATS_WEIYA

#include <nie/core/geometry/isometry3.hpp>
#include <nie/formats/weiya/camera_calibration.hpp>

auto constexpr kFilenameExtrinsicsXml = "/data/aiim/unit_tests_data/formats/weiya/extrinsics.xml";
auto constexpr kFilenameAllBs = "/data/aiim/unit_tests_data/formats/weiya/all.bs";

// We call all transformations T, but as a global using Google style that is butt ugly.
static const nie::Isometry3qd kT_gnss_imu = nie::Isometry3qd::FromTranslation(Eigen::Vector3d{-0.451, -0.038, 0.115});

inline nie::Isometry3qd GetCamToGnssLeft(
    nie::io::weiya::CameraCalibration const& extrinsics_xml, nie::Isometry3qd const& T_imu_l) {
    return kT_gnss_imu * T_imu_l * extrinsics_xml.T_gl;
    // Alternatively, disable to imu to gnss offset (seems worse).
    // return T_imu_l * extrinsics_xml.T_gl;
}

inline nie::Isometry3qd GetCamToGnssRight(
    nie::io::weiya::CameraCalibration const& extrinsics_xml, nie::Isometry3qd const& T_imu_l) {
    return kT_gnss_imu * T_imu_l * extrinsics_xml.T_gl * extrinsics_xml.T_lr;
    // Alternatively, disable to imu to gnss offset (seems worse).
    // return T_imu_l * extrinsics_xml.T_gl * extrinsics_xml.T_lr;
}

#endif  // TEST_FORMATS_WEIYA
