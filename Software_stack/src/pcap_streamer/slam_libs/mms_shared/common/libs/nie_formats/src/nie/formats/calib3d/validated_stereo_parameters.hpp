/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_VALIDATED_STEREO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_VALIDATED_STEREO_PARAMETERS_HPP

#include "base_camera_parameters.hpp"
#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

class ValidatedStereoParameters : public detail::BaseValidatedParameters {
public:
    ValidatedStereoParameters& operator=(ValidatedStereoParameters const&) = delete;

    static ValidatedStereoParameters Read(std::string const& filename);
    static void Write(
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
        std::size_t const& pair_count);

    std::size_t pair_count() const;
    ValidatedCalibrationData const& validated_data_left() const;
    ValidatedCalibrationData const& validated_data_right() const;
    std::vector<std::vector<double>> const& epipolar_errors() const;

private:
    std::size_t pair_count_;
    ValidatedCalibrationData validated_data_left_;
    ValidatedCalibrationData validated_data_right_;
    std::vector<std::vector<double>> epipolar_errors_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_VALIDATED_STEREO_PARAMETERS_HPP
