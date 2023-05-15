/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "camera_calibration.hpp"

#include <glog/logging.h>
#include <opencv2/core.hpp>

namespace nie {
namespace io {
namespace weiya {

CameraCalibration CameraCalibration::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    CHECK(storage.isOpened()) << "Unable to read file: " + filename;

    cv::Mat baseline_translation;
    cv::Mat stereo_correction_rotation_left;
    cv::Mat intrinsics;

    storage["T"] >> baseline_translation;
    storage["R1"] >> stereo_correction_rotation_left;
    storage["P1"] >> intrinsics;

    Eigen::Map<Eigen::Vector3d> t(baseline_translation.ptr<double>());
    Eigen::Map<Eigen::Matrix3d> r(stereo_correction_rotation_left.ptr<double>());
    Eigen::Map<Eigen::Matrix<double, 4, 3>> k(intrinsics.ptr<double>());

    // Normally we take the transpose, because Eigen is column major and OpenCV row major.
    // Now we don't! Because we need the transpose. The matrix is world to camera and we use camera to world.
    Eigen::Quaterniond q{Eigen::Quaterniond{r}};

    // K is transposed because Eigen is column major and OpenCV row major.
    // The corrected baseline is not available. However we know the all stereo calibration information is in the OpenCV
    // frame and that the setup is horizontal. The translation must therefore be the length of the calibrated vector
    // stored in X. Note that the units are in millimeters. Hence the division by 1000.0.
    // Note that the rotation for the right camera equals identity because it has the same rotation as the left one
    // after image correction.
    return {k.block<3, 3>(0, 0).transpose(),
            nie::Isometry3qd::FromRotation(q),
            nie::Isometry3qd::FromTranslation(Eigen::Vector3d{t.norm() / 1000.0, 0.0, 0.0})};
}

}  // namespace weiya
}  // namespace io
}  // namespace nie
