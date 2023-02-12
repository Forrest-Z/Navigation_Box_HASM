/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_CALIBRATED_MONO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_CALIBRATED_MONO_PARAMETERS_HPP

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "base_camera_parameters.hpp"
#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

class CalibratedMonoParameters : public detail::BaseCameraParameters {
public:
    static CalibratedMonoParameters Read(const std::string& filename);
    static void Write(
        std::string const& filename,
        cv::Size const& image_size,
        CalibratedParameters const& parameters,
        std::string const& optics_id,
        std::vector<double> const& intrinsics,
        Eigen::MatrixXd const& cov_intrinsics);

    CalibratedMonoParameters(
        cv::Size const& image_size, CalibratedParameters const& parameters, CalibratedPinhole lens);

    CalibratedPinhole const& lens() const;

private:
    CalibratedMonoParameters() = default;

    CalibratedPinhole lens_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_CALIBRATED_MONO_PARAMETERS_HPP
