/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "extended_mono_parameters.hpp"

namespace nie {

namespace io {

ExtendedMonoParameters ExtendedMonoParameters::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    ExtendedMonoParameters p;
    detail::BaseExtendedParameters::Read(storage, &p);

    storage["calibrated_mono"] >> p.mono_;
    storage["extended_data"] >> p.extended_data_;

    return p;
}

void ExtendedMonoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    CalibratedParameters const& parameters,
    cv::Size const& pattern_size,
    float square_size,
    std::vector<Eigen::Vector3f> const& object_points,
    double var_residuals,
    std::string const& lens_id,
    std::vector<std::string> const& image_ids,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<double> const& intrinsics,
    std::vector<nie::Isometry3qd> const& extrinsics,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals,
    Eigen::MatrixXd const& cov_intrinsics,
    std::vector<Eigen::Matrix<double, 6, 6>> const& cov_extrinsics) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseExtendedParameters::Write(
        image_size, parameters, pattern_size, square_size, object_points, var_residuals, &storage);

    storage << "calibrated_mono" << CalibratedPinhole{lens_id, intrinsics, cov_intrinsics};

    storage << "extended_data"
            << ExtendedCalibrationData{image_ids, image_points, extrinsics, residuals, cov_extrinsics};
}

CalibratedMonoParameters ExtendedMonoParameters::mono_parameters() const {
    return CalibratedMonoParameters(image_size_, distortion_parameters_, mono_);
}

ExtendedCalibrationData const& ExtendedMonoParameters::extended_data() const { return extended_data_; }

}  // namespace io

}  // namespace nie
