/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "validated_stereo_parameters.hpp"

namespace nie {

namespace io {

ValidatedStereoParameters ValidatedStereoParameters::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    ValidatedStereoParameters p;
    detail::BaseValidatedParameters::ReadValidated(storage, &p);

    // opencv doesnt serialize size_t
    int pair_count;
    storage["pair_count"] >> pair_count;
    p.pair_count_ = static_cast<std::size_t>(pair_count);

    storage["validated_data_left"] >> p.validated_data_left_;
    storage["validated_data_right"] >> p.validated_data_right_;

    storage["epipolar_errors"] >> p.epipolar_errors_;

    return p;
}

void ValidatedStereoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::string> const& image_ids_left,
    std::vector<std::string> const& image_ids_right,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_left,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_right,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals_left,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals_right,
    std::vector<std::vector<double>> const& epipolar_errors,
    std::size_t const& pair_count) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseValidatedParameters::WriteValidated(image_size, pattern_size, &storage);

    storage << "pair_count" << static_cast<int>(pair_count);

    storage << "validated_data_left" << ValidatedCalibrationData{image_ids_left, image_points_left, residuals_left};

    storage << "validated_data_right" << ValidatedCalibrationData{image_ids_right, image_points_right, residuals_right};

    storage << "epipolar_errors" << epipolar_errors;
}

std::size_t ValidatedStereoParameters::pair_count() const { return pair_count_; }

ValidatedCalibrationData const& ValidatedStereoParameters::validated_data_left() const { return validated_data_left_; }

ValidatedCalibrationData const& ValidatedStereoParameters::validated_data_right() const {
    return validated_data_right_;
}

std::vector<std::vector<double>> const& ValidatedStereoParameters::epipolar_errors() const { return epipolar_errors_; }

}  // namespace io

}  // namespace nie
