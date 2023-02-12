/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef COMMON_HPP
#define COMMON_HPP

#include <memory>
#include <string>
#include <thread>

#include <nie/core/container/mt_vector.hpp>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/cv/calib3d/distortion_model_parameters.hpp>
#include <nie/formats/calib3d/helper_calibration_io.hpp>
#include <opencv2/core.hpp>

void WriteImage(std::unique_ptr<std::pair<std::string, cv::Mat>> const& work);

bool ReadImage(std::string const& in_path_image, std::pair<std::string, cv::Mat>* image);

bool DetectCorners(cv::Mat const& image, cv::Size const& pattern_size, std::vector<Eigen::Vector2f>* corners);

using ImageCorners = std::pair<std::string, std::vector<Eigen::Vector2f>>;

void GetCornerData(
    std::string const& in_path_images,
    cv::Size const& pattern_size,
    bool write_corner_images,
    nie::mt::MtVector<ImageCorners>* p_ids_and_image_points);

void GetCornerData(
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    cv::Size const& pattern_size,
    bool write_corner_images,
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>>* p_ids_and_image_points_pairs,
    nie::mt::MtVector<ImageCorners>* p_ids_and_image_points_left,
    nie::mt::MtVector<ImageCorners>* p_ids_and_image_points_right);

void MoveStereoInput(
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>>* ids_and_image_points_pairs,
    nie::mt::MtVector<ImageCorners>* ids_and_image_points_left,
    nie::mt::MtVector<ImageCorners>* ids_and_image_points_right,
    std::vector<std::vector<Eigen::Vector2f>>* image_points_left,
    std::vector<std::vector<Eigen::Vector2f>>* image_points_right,
    std::vector<std::string>* ids_left,
    std::vector<std::string>* ids_right);

struct CornerDetector {
    static std::uint32_t GetProcessorCountCorners() {
        std::uint32_t processor_count = std::thread::hardware_concurrency();
        return static_cast<std::uint32_t>(std::floor((static_cast<float>(processor_count) / 3.0f) * 2.0f));
    }

    static std::uint32_t GetProcessorCountReader() {
        std::uint32_t processor_count = std::thread::hardware_concurrency();
        return static_cast<std::uint32_t>(std::ceil(static_cast<float>(processor_count) / 3.0f));
    }
};

inline Eigen::Matrix3d GetRotationOrthogonalizingBaselineAsXCw(Eigen::Vector3d const& baseline) {
    // Can't orthogonalize with just a Z vector
    assert(baseline.head<2>().squaredNorm() > 0);
    Eigen::Vector3d x = baseline;
    Eigen::Vector3d z(0.0, 0.0, 1.0);
    Eigen::Vector3d y = z.cross(x);
    z = x.cross(y);

    x.normalize();
    y.normalize();
    z.normalize();

    Eigen::Matrix3d Ro;
    Ro << x.x(), x.y(), x.z(), y.x(), y.y(), y.z(), z.x(), z.y(), z.z();
    return Ro;
}

inline void GetRectifiedParametersStereoCw(
    nie::Isometry3qd const& baseline,
    Eigen::Vector3d* p_baseline_t_c,
    Eigen::Matrix3d* p_R_c_left,
    Eigen::Matrix3d* p_R_c_right) {
    nie::Isometry3qd baseline_inv = baseline.Inversed();
    Eigen::Matrix3d R_c_left = GetRotationOrthogonalizingBaselineAsXCw(baseline_inv.translation());
    Eigen::Matrix3d R_c_right = R_c_left * baseline_inv.rotation();
    *p_baseline_t_c = -R_c_left * baseline_inv.translation();
    *p_R_c_left = R_c_left;
    *p_R_c_right = R_c_right;
}

// Helpers to contain the madness, can't be placed together to the
// helper_distortion_parameters_io because then we would bring back
// the circular dependency.
inline nie::io::CalibratedParameters CalibratedParametersFromDistortionParameters(
    nie::DistortionModelParameters const& parameters) {
    return nie::io::CalibratedParameters{nie::ParameterFocalLengthToString(parameters.focal_length),
                                         nie::ParameterSkewToString(parameters.skew),
                                         nie::ParameterDistortionRadialToString(parameters.distortion_radial),
                                         nie::ParameterDistortionTangentialToString(parameters.distortion_tangential),
                                         nie::ParameterDistortionThinPrismToString(parameters.distortion_thin_prism)};
}

inline nie::DistortionModelParameters DistortionParametersFromCalibratedParameters(
    nie::io::CalibratedParameters const& parameters) {
    return nie::DistortionModelParameters{nie::ParameterFocalLengthToEnum(parameters.focal_length),
                                          nie::ParameterSkewToEnum(parameters.skew),
                                          nie::ParameterDistortionRadialToEnum(parameters.distortion_radial),
                                          nie::ParameterDistortionTangentialToEnum(parameters.distortion_tangential),
                                          nie::ParameterDistortionThinPrismToEnum(parameters.distortion_thin_prism)};
}

#endif  // COMMON_HPP
