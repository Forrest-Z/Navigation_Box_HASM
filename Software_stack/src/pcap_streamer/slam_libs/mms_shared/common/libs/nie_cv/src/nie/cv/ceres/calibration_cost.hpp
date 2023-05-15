/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_CALIBRATION_COST_HPP
#define NIE_CV_CERES_CALIBRATION_COST_HPP

#include <ceres/ceres.h>
#include <opencv2/core.hpp>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"
#include "nie/cv/ceres/helper_ceres.hpp"

namespace nie {

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeCalibrationCost {
public:
    static constexpr int kParametersIntrinsics = detail::kParametersIntrinsics<FL, SK, RA, TA, TP>;
    static constexpr int kParametersTranslation = detail::kParamatersTranslation;
    static constexpr int kParametersRotation = detail::kParametersRotation;

    PinholeCalibrationCost(Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point)
        : object_point_(object_point.cast<double>()), image_point_(image_point.cast<double>()) {}

    static ceres::CostFunction* Create(Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) {
        return new ceres::AutoDiffCostFunction<
            PinholeCalibrationCost<FL, SK, RA, TA, TP>,
            2,  // Residuals
            kParametersIntrinsics,
            kParametersTranslation,
            kParametersRotation>(new PinholeCalibrationCost<FL, SK, RA, TA, TP>(object_point, image_point));
    }

    template <typename T>
    bool operator()(
        T const* const intrinsics, T const* const translation, T const* const rotation, T* residuals) const {
        Eigen::Matrix<T, 3, 1> object_point = object_point_.cast<T>();
        Eigen::Matrix<T, 2, 1> image_point = image_point_.cast<T>();
        detail::CalculateBackProjectionError<FL, SK, RA, TA, TP>(
            Isometry3Map<const Eigen::Quaternion<T>>{translation, rotation},
            intrinsics,
            object_point.template data(),
            image_point.template data(),
            residuals);
        return true;
    }

private:
    Eigen::Vector3d object_point_;
    Eigen::Vector2d image_point_;
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeWithBaselineCalibrationCost {
public:
    static constexpr int kParametersIntrinsics = detail::kParametersIntrinsics<FL, SK, RA, TA, TP>;
    static constexpr int kParametersTranslation = detail::kParamatersTranslation;
    static constexpr int kParametersRotation = detail::kParametersRotation;

    PinholeWithBaselineCalibrationCost(Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point)
        : object_point_(object_point.cast<double>()), image_point_(image_point.cast<double>()) {}

    static ceres::CostFunction* Create(Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) {
        return new ceres::AutoDiffCostFunction<
            PinholeWithBaselineCalibrationCost<FL, SK, RA, TA, TP>,
            2,
            kParametersIntrinsics,
            kParametersTranslation,
            kParametersRotation,
            kParametersTranslation,
            kParametersRotation>(new PinholeWithBaselineCalibrationCost<FL, SK, RA, TA, TP>(object_point, image_point));
    }

    template <typename T>
    bool operator()(
        const T* const intrinsics,
        // reference extrinsics
        const T* const e_translation,
        const T* const e_rotation,
        // baseline
        const T* const b_translation,
        const T* const b_rotation,
        T* residuals) const {
        auto extrinsics = Isometry3Map<const Eigen::Quaternion<T>>{e_translation, e_rotation} *
                          Isometry3Map<const Eigen::Quaternion<T>>{b_translation, b_rotation};
        Eigen::Matrix<T, 3, 1> object_point = object_point_.cast<T>();
        Eigen::Matrix<T, 2, 1> image_point = image_point_.cast<T>();
        detail::CalculateBackProjectionError<FL, SK, RA, TA, TP>(
            extrinsics, intrinsics, object_point.template data(), image_point.template data(), residuals);
        return true;
    }

private:
    Eigen::Vector3d object_point_;
    Eigen::Vector2d image_point_;
};

}  // namespace nie

#endif  // NIE_CV_CERES_CALIBRATION_COST_HPP
