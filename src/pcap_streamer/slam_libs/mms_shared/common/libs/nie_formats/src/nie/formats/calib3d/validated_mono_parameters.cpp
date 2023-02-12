/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "validated_mono_parameters.hpp"

namespace nie {

namespace io {

ValidatedMonoParameters ValidatedMonoParameters::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    ValidatedMonoParameters p;
    detail::BaseValidatedParameters::ReadValidated(storage, &p);

    storage["validated_data"] >> p.validated_data_;

    return p;
}

void ValidatedMonoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::string> const& image_ids,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseValidatedParameters::WriteValidated(image_size, pattern_size, &storage);

    storage << "validated_data" << ValidatedCalibrationData{image_ids, image_points, residuals};
}

ValidatedCalibrationData const& ValidatedMonoParameters::validated_data() const { return validated_data_; }

}  // namespace io

}  // namespace nie
