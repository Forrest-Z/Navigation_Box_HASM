/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_BASE_CAMERA_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_BASE_CAMERA_PARAMETERS_HPP

#include <opencv2/core.hpp>

#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

namespace detail {

// TODO(jbr) Maybe add a date and/or version or camera id
class BaseCameraParameters {
public:
    BaseCameraParameters() = default;
    BaseCameraParameters(cv::Size const& image_size, CalibratedParameters const& parameters)
        : image_size_(image_size), distortion_parameters_(parameters) {}

    cv::Size image_size() const;
    CalibratedParameters distortion_parameters() const;

protected:
    static void Read(cv::FileStorage const& storage, BaseCameraParameters* p);
    static void Write(cv::Size const& image_size, CalibratedParameters const& parameters, cv::FileStorage* storage);

    cv::Size image_size_;
    CalibratedParameters distortion_parameters_;
};

class BaseExtendedParameters : public BaseCameraParameters {
public:
    cv::Size pattern_size() const;
    float square_size() const;
    std::vector<Eigen::Vector3f> const& object_points() const;
    double var_residuals() const;

protected:
    static void Read(cv::FileStorage const& storage, BaseExtendedParameters* p);
    static void Write(
        cv::Size const& image_size,
        CalibratedParameters const& parameters,
        cv::Size const& pattern_size,
        float square_size,
        std::vector<Eigen::Vector3f> const& object_points,
        double var_residuals,
        cv::FileStorage* storage);

    cv::Size pattern_size_;
    float square_size_;
    std::vector<Eigen::Vector3f> object_points_;
    double var_residuals_;
};

class BaseValidatedParameters {
public:
    static void ReadValidated(cv::FileStorage const& storage, BaseValidatedParameters* p);
    static void WriteValidated(cv::Size const& image_size, cv::Size const& pattern_size, cv::FileStorage* storage);

    cv::Size image_size() const;
    cv::Size pattern_size() const;

protected:
    cv::Size image_size_;
    cv::Size pattern_size_;
};

}  // namespace detail

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_BASE_CAMERA_PARAMETERS_HPP
