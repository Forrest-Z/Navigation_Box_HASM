/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_WEIYA_RECTIFIED_PARAMETERS_HPP
#define NIE_FORMATS_WEIYA_RECTIFIED_PARAMETERS_HPP

#include <glog/logging.h>

#include "../calib3d/rectified_parameters.hpp"
#include "camera_boresight.hpp"
#include "camera_calibration.hpp"

namespace nie {
namespace io {
namespace weiya {

/// Reads Weiya input files and converts it to our own RectifiedCameraParameters.
inline nie::io::RectifiedCameraParameters RectifiedCameraParametersFromWeiya(
    std::string const& filename_calibration, std::string const& filename_boresight) {
    auto calibration = CameraCalibration::Read(filename_calibration);
    auto T_imu_cam = CameraBoresight::Read(filename_boresight);

    LOG(WARNING) << "nie::io::weiya::RectifiedCameraParametersFromWeiya(): Using hardcoded 'IMU to GNSS Antenna Lever "
                    "Arms' from PosT header.";
    // There is no provision for reading the PosT header.
    auto T_gnss_imu = nie::Isometry3qd::FromTranslation(Eigen::Vector3d{-0.451, -0.038, 0.115});

    LOG(WARNING) << "nie::io::weiya::RectifiedCameraParametersFromWeiya(): Using hardcoded image resolution.";
    return {"weiya",
            // This cannot be read from any file and is always fixed (or at least that seems to be the case).
            Eigen::Vector2i{4096, 2168},
            calibration.K,
            {{"left", T_gnss_imu * T_imu_cam * calibration.T_gl},
             {"right", T_gnss_imu * T_imu_cam * calibration.T_gl * calibration.T_lr}}};
}

}  // namespace weiya
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_WEIYA_RECTIFIED_PARAMETERS_HPP
