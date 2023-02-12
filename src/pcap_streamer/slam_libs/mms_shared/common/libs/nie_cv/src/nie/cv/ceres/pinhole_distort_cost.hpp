/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_PINHOLE_DISTORT_COST_HPP
#define NIE_CV_CERES_PINHOLE_DISTORT_COST_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"
#include "nie/cv/ceres/helper_ceres.hpp"

namespace nie {

//
// Find a distorted point d by minimizing || d - Distort(u) ||^2 over the intrinsics and the distorted point.
//
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeDistortWithIntrinsicsCost {
public:
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

private:
    using Jet = ceres::Jet<double, 2 + kParametersIntrinsics>;

public:
    PinholeDistortWithIntrinsicsCost(
        std::vector<double> const& intrinsics_from_K_only, Eigen::Vector2d const& image_point_undistorted)
        : intrinsics_from_K_only_v_(intrinsics_from_K_only),
          intrinsics_from_K_only_J_(intrinsics_from_K_only.size()),
          image_point_undistorted_(image_point_undistorted) {
        CHECK(intrinsics_from_K_only.size() == kParametersIntrinsics);

        for (std::size_t i = 0; i < intrinsics_from_K_only.size(); ++i) {
            intrinsics_from_K_only_J_[i] = Jet(intrinsics_from_K_only_v_[i]);
        }
    }

    static ceres::CostFunction* Create(
        std::vector<double> const& intrinsics_from_K_only, Eigen::Vector2d const& image_point_undistorted) {
        return new ceres::
            AutoDiffCostFunction<PinholeDistortWithIntrinsicsCost<FL, SK, RA, TA, TP>, 2, kParametersIntrinsics, 2>(
                new PinholeDistortWithIntrinsicsCost<FL, SK, RA, TA, TP>(
                    intrinsics_from_K_only, image_point_undistorted));
    }

    template <typename T>
    bool operator()(T const* const intrinsics, T const* const distorted, T* residuals) const {
        T p_u;
        T p_v;
        UndistortedImageToDistortedImage(
            intrinsics, T(image_point_undistorted_[0]), T(image_point_undistorted_[1]), &p_u, &p_v);

        residuals[0] = distorted[0] - p_u;
        residuals[1] = distorted[1] - p_v;

        return true;
    }

private:
    void UndistortedImageToDistortedImage(
        double const* const intrinsics_distorted,
        double const& pu_u,
        double const& pu_v,
        double* pd_u,
        double* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_from_K_only_v_.data(), intrinsics_distorted, pu_u, pu_v, pd_u, pd_v);
    }

    void UndistortedImageToDistortedImage(
        Jet const* const intrinsics_distorted, Jet const& pu_u, Jet const& pu_v, Jet* pd_u, Jet* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_from_K_only_J_.data(), intrinsics_distorted, pu_u, pu_v, pd_u, pd_v);
    }

    std::vector<double> intrinsics_from_K_only_v_;
    // These are constants and the size doesn't change depending on the camera model.
    // As an optimization we pre-allocate everything instead of the typical "casting"
    // to type T (double to Jet constant constructor).
    std::vector<Jet> intrinsics_from_K_only_J_;

    Eigen::Vector2d image_point_undistorted_;
};

}  // namespace nie

#endif  // NIE_CV_CERES_PINHOLE_DISTORT_COST_HPP
