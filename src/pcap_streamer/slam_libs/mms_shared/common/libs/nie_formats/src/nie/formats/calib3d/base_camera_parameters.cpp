/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "base_camera_parameters.hpp"

namespace nie {

namespace io {

namespace detail {

// Moved the distortion params I/O logic here from helper_calibration_io
// to avoid having a circular dependency there
void BaseCameraParameters::Read(cv::FileStorage const& storage, BaseCameraParameters* p) {
    storage["image_size"] >> p->image_size_;
    storage["distortion_model"] >> p->distortion_parameters_;
}

void BaseCameraParameters::Write(
    cv::Size const& image_size, CalibratedParameters const& parameters, cv::FileStorage* storage) {
    *storage << "image_size" << image_size;
    *storage << "distortion_model" << parameters;
}

cv::Size BaseCameraParameters::image_size() const { return image_size_; }

CalibratedParameters BaseCameraParameters::distortion_parameters() const { return distortion_parameters_; }

cv::Size BaseExtendedParameters::pattern_size() const { return pattern_size_; }

float BaseExtendedParameters::square_size() const { return square_size_; }

std::vector<Eigen::Vector3f> const& BaseExtendedParameters::object_points() const { return object_points_; }

double BaseExtendedParameters::var_residuals() const { return var_residuals_; }

void BaseExtendedParameters::Read(cv::FileStorage const& storage, BaseExtendedParameters* p) {
    BaseCameraParameters::Read(storage, p);
    storage["pattern_size"] >> p->pattern_size_;
    storage["square_size"] >> p->square_size_;
    storage["object_points"] >> p->object_points_;
    storage["var_residuals"] >> p->var_residuals_;
}

void BaseExtendedParameters::Write(
    cv::Size const& image_size,
    CalibratedParameters const& parameters,
    cv::Size const& pattern_size,
    float square_size,
    std::vector<Eigen::Vector3f> const& object_points,
    double var_residuals,
    cv::FileStorage* storage) {
    BaseCameraParameters::Write(image_size, parameters, storage);
    *storage << "pattern_size" << pattern_size;
    *storage << "square_size" << square_size;
    *storage << "object_points" << object_points;
    *storage << "var_residuals" << var_residuals;
}

void BaseValidatedParameters::ReadValidated(cv::FileStorage const& storage, BaseValidatedParameters* p) {
    storage["image_size"] >> p->image_size_;
    storage["pattern_size"] >> p->pattern_size_;
}

void BaseValidatedParameters::WriteValidated(
    cv::Size const& image_size, cv::Size const& pattern_size, cv::FileStorage* storage) {
    *storage << "image_size" << image_size;
    *storage << "pattern_size" << pattern_size;
}

cv::Size BaseValidatedParameters::image_size() const { return image_size_; }

cv::Size BaseValidatedParameters::pattern_size() const { return pattern_size_; }

}  // namespace detail

}  // namespace io

}  // namespace nie
