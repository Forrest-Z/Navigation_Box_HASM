/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "calibrated_mono_parameters.hpp"

namespace nie {

namespace io {

CalibratedMonoParameters CalibratedMonoParameters::Read(const std::string& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    CalibratedMonoParameters p;
    detail::BaseCameraParameters::Read(storage, &p);

    storage["calibrated_optics"] >> p.lens_;

    return p;
}

void CalibratedMonoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    CalibratedParameters const& parameters,
    std::string const& optics_id,
    std::vector<double> const& intrinsics,
    Eigen::MatrixXd const& cov_intrinsics) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseCameraParameters::Write(image_size, parameters, &storage);

    storage << "calibrated_optics" << CalibratedPinhole{optics_id, intrinsics, cov_intrinsics};
}

CalibratedMonoParameters::CalibratedMonoParameters(
    cv::Size const& image_size, CalibratedParameters const& parameters, CalibratedPinhole lens)
    : BaseCameraParameters(image_size, parameters), lens_(std::move(lens)) {}

CalibratedPinhole const& CalibratedMonoParameters::lens() const { return lens_; }

}  // namespace io

}  // namespace nie
