/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "camera_boresight.hpp"

#include <glog/logging.h>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace nie {
namespace io {
namespace weiya {

nie::Isometry3qd CameraBoresight::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    CHECK(storage.isOpened()) << "Unable to read file: " + filename;

    cv::Mat scale;
    cv::Mat translation;
    cv::Mat rotation;

    // The all.bs file can contain a scale that, for now, just seems to be 1.0. However, when it isn't we don't know
    // how to handle it yet.
    storage["Scale"] >> scale;
    CHECK(*scale.ptr<double>() == 1.0)
        << "CameraBoresight::Read() does not support a Scale value that is different from 1.0.";

    storage["TranslationVectorCam2IMU"] >> translation;
    storage["RotateMatrixCam2IMU"] >> rotation;

    Eigen::Map<Eigen::Vector3d> t(translation.ptr<double>());
    Eigen::Map<Eigen::Matrix3d> r(rotation.ptr<double>());

    // We transpose the rotation because Eigen is column major and OpenCV row major.
    return nie::Isometry3qd{t, Eigen::Quaterniond{r.transpose()}};
}

}  // namespace weiya
}  // namespace io
}  // namespace nie
