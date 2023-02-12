/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_VALIDATED_MONO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_VALIDATED_MONO_PARAMETERS_HPP

#include "base_camera_parameters.hpp"
#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

class ValidatedMonoParameters : public detail::BaseValidatedParameters {
public:
    ValidatedMonoParameters& operator=(ValidatedMonoParameters const&) = delete;

    static ValidatedMonoParameters Read(std::string const& filename);
    static void Write(
        std::string const& filename,
        cv::Size const& image_size,
        cv::Size const& pattern_size,
        std::vector<std::string> const& image_ids,
        std::vector<std::vector<Eigen::Vector2f>> const& image_points,
        std::vector<std::vector<Eigen::Vector2f>> const& residuals);

    ValidatedCalibrationData const& validated_data() const;

private:
    ValidatedCalibrationData validated_data_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_VALIDATED_MONO_PARAMETERS_HPP
