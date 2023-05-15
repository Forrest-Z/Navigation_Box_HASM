/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_CALIBRATED_STEREO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_CALIBRATED_STEREO_PARAMETERS_HPP

#include <memory>
#include <string>

#include "base_camera_parameters.hpp"
#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

class CalibratedStereoParameters : public detail::BaseCameraParameters {
public:
    static CalibratedStereoParameters Read(const std::string& filename);
    static void Write(
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
        Eigen::Matrix<double, 6, 6> const& cov_baseline);

    CalibratedStereoParameters(
        cv::Size const& image_size, CalibratedParameters const& parameters, CalibratedStereo stereo);

    CalibratedPinhole const& lens_left() const;
    CalibratedPinhole const& lens_right() const;
    CalibratedBaseline const& baseline() const;

private:
    CalibratedStereoParameters() = default;

    CalibratedStereo stereo_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_CALIBRATED_STEREO_PARAMETERS_HPP
