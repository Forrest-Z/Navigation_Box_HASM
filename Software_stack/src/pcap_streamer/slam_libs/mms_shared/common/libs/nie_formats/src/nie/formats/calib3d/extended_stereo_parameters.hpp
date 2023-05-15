/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_EXTENDED_STEREO_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_EXTENDED_STEREO_PARAMETERS_HPP

#include <opencv2/core.hpp>
#include <string>

#include "calibrated_stereo_parameters.hpp"

namespace nie {

namespace io {

class ExtendedStereoParameters : public detail::BaseExtendedParameters {
public:
    ExtendedStereoParameters& operator=(ExtendedStereoParameters const&) = delete;

    static ExtendedStereoParameters Read(std::string const& filename);
    static void Write(
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
        Eigen::Matrix<double, 6, 6> const& cov_baseline);

    std::size_t pair_count() const;
    CalibratedStereoParameters stereo_parameters() const;
    ExtendedCalibrationData const& extended_data_left() const;
    ExtendedCalibrationData const& extended_data_right() const;

private:
    std::size_t pair_count_;
    CalibratedStereo stereo_;
    ExtendedCalibrationData extended_data_left_;
    ExtendedCalibrationData extended_data_right_;
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_EXTENDED_STEREO_PARAMETERS_HPP
