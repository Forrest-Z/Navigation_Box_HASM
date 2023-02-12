/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "calibrated_stereo_parameters.hpp"

namespace nie {

namespace io {

CalibratedStereoParameters CalibratedStereoParameters::Read(const std::string& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    CalibratedStereoParameters p;
    detail::BaseCameraParameters::Read(storage, &p);

    storage["calibrated_stereo"] >> p.stereo_;

    return p;
}

void CalibratedStereoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    CalibratedParameters const& parameters,
    std::string const& optics_id_left,
    std::vector<double> const& intrinsics_left,
    Eigen::MatrixXd const& cov_intrinsics_left,
    std::string const& optics_id_right,
    std::vector<double> const& intrinsics_right,
    Eigen::MatrixXd const& cov_intrinsics_right,
    nie::Isometry3qd const& baseline,
    Eigen::Matrix<double, 6, 6> const& cov_baseline) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseCameraParameters::Write(image_size, parameters, &storage);

    storage << "calibrated_stereo";
    storage << CalibratedStereo{{optics_id_left, intrinsics_left, cov_intrinsics_left},
                                {optics_id_right, intrinsics_right, cov_intrinsics_right},
                                {baseline, cov_baseline}};
}

CalibratedStereoParameters::CalibratedStereoParameters(
    cv::Size const& image_size, CalibratedParameters const& parameters, CalibratedStereo stereo)
    : BaseCameraParameters(image_size, parameters), stereo_(std::move(stereo)) {}

CalibratedPinhole const& CalibratedStereoParameters::lens_left() const { return stereo_.lens_left; }

CalibratedPinhole const& CalibratedStereoParameters::lens_right() const { return stereo_.lens_right; }

CalibratedBaseline const& CalibratedStereoParameters::baseline() const { return stereo_.baseline; }

}  // namespace io

}  // namespace nie
