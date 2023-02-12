/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_EXTENDED_MONO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_EXTENDED_MONO_PARAMETERS_HPP

#include <opencv2/core.hpp>
#include <string>

#include "calibrated_mono_parameters.hpp"

namespace nie {

namespace io {

class ExtendedMonoParameters : public detail::BaseExtendedParameters {
public:
    ExtendedMonoParameters& operator=(ExtendedMonoParameters const&) = delete;

    static ExtendedMonoParameters Read(std::string const& filename);
    static void Write(
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
        std::vector<Eigen::Matrix<double, 6, 6>> const& cov_extrinsics);

    CalibratedMonoParameters mono_parameters() const;
    ExtendedCalibrationData const& extended_data() const;

private:
    CalibratedPinhole mono_;
    ExtendedCalibrationData extended_data_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_EXTENDED_MONO_PARAMETERS_HPP
