/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_WEIYA_CAMERA_CALIBRATION_HPP
#define NIE_FORMATS_WEIYA_CAMERA_CALIBRATION_HPP

#include <Eigen/Geometry>
#include <nie/core/geometry/isometry3.hpp>

namespace nie {

namespace io {

namespace weiya {

/// The Extrinsics XML contains all outputs of the OpenCV stereo calibration method.
/// See: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereocalibrate
///
/// However, we are only interested in the rectified camera intrinsics and rectified camera transformations.
/// All transformations are local to global (camera to their respective world).
/// Left to world, right goes to left.
/// Thus, all translations are in "global".
class CameraCalibration {
public:
    static CameraCalibration Read(std::string const& filename);

    Eigen::Matrix3d K;
    /// Left (or local) to global (which is strictly speaking unknown - in practice the imu).
    nie::Isometry3qd T_gl;
    /// Right to left
    nie::Isometry3qd T_lr;
};

}  // namespace weiya

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_WEIYA_CAMERA_CALIBRATION_HPP
