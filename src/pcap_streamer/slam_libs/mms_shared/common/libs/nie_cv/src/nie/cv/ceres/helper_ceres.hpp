/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_HELPER_CERES_HPP
#define NIE_CV_CERES_HELPER_CERES_HPP

#include <ceres/ceres.h>
#include <nie/core/geometry/isometry3.hpp>
#include <opencv2/core.hpp>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"

namespace nie {

namespace detail {

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
constexpr int kParametersIntrinsics = detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;
constexpr int kParamatersTranslation = 3;
constexpr int kParametersRotation = 4;
constexpr int kParametersObjectPoint = 3;

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP,
    typename T>
void UndistortedImageToDistortedImage(
    const T* const intrinsics_undistorted,
    const T* const intrinsics_distorted,
    T const& pu_u,
    T const& pu_v,
    T* pd_u,
    T* pd_v) {
    T vu_u;
    T vu_v;
    detail::BehaviorFocalLength<FL>::Undo(intrinsics_undistorted, pu_u, pu_v, &vu_u, &vu_v);
    detail::BehaviorSkew<SK>::Undo(intrinsics_undistorted, vu_u, vu_v, &vu_u, &vu_v);
    detail::Distort<FL, SK, RA, TA, TP>(intrinsics_distorted, vu_u, vu_v, pd_u, pd_v);
}

/// @brief Full model projection error.
/// @param translation 3 elements
/// @param rotation 4 elements quaternion
/// @param intrinsics 4 elements => focal length and principal point
/// @param object_point 3 elements
/// @param image_point 2 elements
/// @param residuals 2 elements
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP,
    typename T,
    typename Isometry,
    typename Rotation>
inline void CalculateBackProjectionError(
    Isometry3Base<Isometry, Rotation> const& extrinsics,
    T const* const intrinsics,
    T const* const object_point,
    T const* const image_point,
    T* residuals) {
    // Camera vector.
    Eigen::Matrix<T, 3, 1> v = extrinsics.TransformInverseLeft(Eigen::Map<const Eigen::Matrix<T, 3, 1>>{object_point});
    // Divide by Z to get the undistorted normalized coordinates.
    Eigen::Matrix<T, 2, 1> u = v.hnormalized();
    Eigen::Matrix<T, 2, 1> p;
    detail::Distort<FL, SK, RA, TA, TP>(intrinsics, u.data()[0], u.data()[1], &p.data()[0], &p.data()[1]);
    // Final error
    Eigen::Map<Eigen::Matrix<T, 2, 1>>{residuals} = p - Eigen::Map<const Eigen::Matrix<T, 2, 1>>{image_point};
}

/// @brief Linear model projection error.
/// @param translation 3 elements
/// @param rotation 4 elements quaternion
/// @param intrinsics 4 elements => focal length and principal point
/// @param object_point 3 elements
/// @param image_point 2 elements
/// @param residuals 2 elements
template <ParameterFocalLength FL, ParameterSkew SK, typename T, typename Isometry, typename Rotation>
inline void CalculateBackProjectionError(
    Isometry3Base<Isometry, Rotation> const& extrinsics,
    T const* const intrinsics,
    T const* const object_point,
    T const* const image_point,
    T* residuals) {
    return CalculateBackProjectionError<
        FL,
        SK,
        ParameterDistortionRadial::NO_DISTORTION,
        ParameterDistortionTangential::NO_DISTORTION,
        ParameterDistortionThinPrism::NO_DISTORTION>(extrinsics, intrinsics, object_point, image_point, residuals);
}

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_CERES_HELPER_CERES_HPP
