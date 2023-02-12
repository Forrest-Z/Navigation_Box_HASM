/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "extended_stereo_parameters.hpp"

#include <opencv2/core/persistence.hpp>

namespace nie {

namespace io {

ExtendedStereoParameters ExtendedStereoParameters::Read(std::string const& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to read file: " + filename);
    }

    ExtendedStereoParameters p;
    detail::BaseExtendedParameters::Read(storage, &p);

    // opencv doesnt serialize size_t
    int pair_count;
    storage["pair_count"] >> pair_count;
    p.pair_count_ = static_cast<std::size_t>(pair_count);

    storage["calibrated_stereo"] >> p.stereo_;
    storage["extended_data_left"] >> p.extended_data_left_;
    storage["extended_data_right"] >> p.extended_data_right_;

    return p;
}

void ExtendedStereoParameters::Write(
    std::string const& filename,
    cv::Size const& image_size,
    CalibratedParameters const& parameters,
    cv::Size const& pattern_size,
    float square_size,
    std::vector<Eigen::Vector3f> const& object_points,
    double var_residuals,
    std::string const& optics_id_left,
    std::string const& optics_id_right,
    std::vector<std::string> const& image_ids_left,
    std::vector<std::string> const& image_ids_right,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_left,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_right,
    std::vector<double> const& intrinsics_left,
    std::vector<double> const& intrinsics_right,
    std::vector<nie::Isometry3qd> const& extrinsics_left,
    std::vector<nie::Isometry3qd> const& extrinsics_right,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals_left,
    std::vector<std::vector<Eigen::Vector2f>> const& residuals_right,
    Eigen::MatrixXd const& cov_intrinsics_left,
    Eigen::MatrixXd const& cov_intrinsics_right,
    std::vector<Eigen::Matrix<double, 6, 6>> const& cov_extrinsics_left,
    std::vector<Eigen::Matrix<double, 6, 6>> const& cov_extrinsics_right,
    std::size_t const& pair_count,
    nie::Isometry3qd const& baseline,
    Eigen::Matrix<double, 6, 6> const& cov_baseline) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);

    if (!storage.isOpened()) {
        throw std::runtime_error("Unable to write file: " + filename);
    }

    detail::BaseExtendedParameters::Write(
        image_size, parameters, pattern_size, square_size, object_points, var_residuals, &storage);

    storage << "pair_count" << static_cast<int>(pair_count);

    storage << "calibrated_stereo"
            << CalibratedStereo{{optics_id_left, intrinsics_left, cov_intrinsics_left},
                                {optics_id_right, intrinsics_right, cov_intrinsics_right},
                                {baseline, cov_baseline}};

    storage << "extended_data_left"
            << ExtendedCalibrationData{
                   image_ids_left, image_points_left, extrinsics_left, residuals_left, cov_extrinsics_left};

    storage << "extended_data_right"
            << ExtendedCalibrationData{
                   image_ids_right, image_points_right, extrinsics_right, residuals_right, cov_extrinsics_right};
}

std::size_t ExtendedStereoParameters::pair_count() const { return pair_count_; }

CalibratedStereoParameters ExtendedStereoParameters::stereo_parameters() const {
    return CalibratedStereoParameters(image_size_, distortion_parameters_, stereo_);
}

ExtendedCalibrationData const& ExtendedStereoParameters::extended_data_left() const { return extended_data_left_; }

ExtendedCalibrationData const& ExtendedStereoParameters::extended_data_right() const { return extended_data_right_; }

}  // namespace io

}  // namespace nie
