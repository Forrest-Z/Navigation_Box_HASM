/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_CAMERA_CALIBRATION_HPP
#define NIE_CV_CALIB3D_CAMERA_CALIBRATION_HPP

#include <thread>
#include <vector>

#include <nie/core/geometry/isometry3.hpp>
#include <opencv2/core.hpp>

#include "distortion_model_factory.hpp"

namespace nie {

// Camera calibration handles estimating of camera intrinsics and extrinsics using object to
// image point correspondences of a planar object. It is assumed that all points in the plane
// have z=0.0
//
// Method implemented according to Zhengyou Zhang's paper:
//      A Flexible New Technique for Camera Calibration
//
// The output translations are determined based on the input object (plane) points and share
// the same units. It is assumed that a right-handed coordinate system is used for the object
// points:
//
//      { X-east, Y-down, Z-north }
//
// The image or camera space is considered identical.

// *******************************************************************************************

// TODO(jbr): A more Eigen oriented interface. Not some hybrid like this.

/// @brief A non-linear estimation of camera intrinsics and extrinsics using Zhang's method as an initial estimation.
/// @details Outputs extrinsics in the camera to world convention.
// Rotation matrices bring camera vectors into world referenced vectors: V_w = R * V_c;
//
// The full projection can be handled as:
//
//      p = K * R^T(X_w - t)
//
// The result is up to scale (divide by the Z of the result). Similar with a 4x4 matrix:
//
//          [ K*R^T  | -K*R^T*t ]
//      p = [ 0^T    | 1        ] [ X_w, 1]
//
// The position of each camera is simply: t
//
void EstimateCameraParametersCeres(
    DistortionModelParameters const& parameters,
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<double>* p_intrinsics,
    std::vector<nie::Isometry3qd>* p_extrinsics,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals,
    Eigen::MatrixXd* p_cov_intrinsics,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics,
    double* p_var_residuals,
    const unsigned int num_threads = std::thread::hardware_concurrency());

/// @refitem EstimateCameraParametersCeres
void EstimateStereoParametersCeres(
    DistortionModelParameters const& parameters,
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_left,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_right,
    std::size_t const& pair_count,
    std::vector<double>* p_intrinsics_left,
    std::vector<double>* p_intrinsics_right,
    std::vector<nie::Isometry3qd>* p_extrinsics_left,
    // The translations of the right camera that aren't estimated with respect to the left one
    // are stored here. The size of the vector equals image_points_right.size() - pair_count
    std::vector<nie::Isometry3qd>* p_extrinsics_right,
    nie::Isometry3qd* p_baseline,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals_left,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals_right,
    Eigen::MatrixXd* p_cov_intrinsics_left,
    Eigen::MatrixXd* p_cov_intrinsics_right,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics_left,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics_right,
    Eigen::Matrix<double, 6, 6>* p_cov_baseline,
    double* p_var_residuals,
    const unsigned int num_threads = std::thread::hardware_concurrency());

}  // namespace nie

#endif  // NIE_CV_CALIB3D_CAMERA_CALIBRATION_HPP
