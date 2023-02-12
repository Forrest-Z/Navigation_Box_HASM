/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_UNDISTORT_COST_HPP
#define NIE_CV_CERES_UNDISTORT_COST_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"
#include "nie/cv/ceres/helper_ceres.hpp"

namespace nie {

//
// Find an undistorted u point by minimizing || d - Distort(u) ||^2 over the undistorted point
// The intrinsics are considered constants.
//
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeUndistortCost {
public:
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

private:
    using Jet = ceres::Jet<double, 2>;

public:
    PinholeUndistortCost(
        std::vector<double> const& intrinsics_from,
        std::vector<double> const& intrinsics_to_K_only,
        Eigen::Vector2d const& image_point)
        : intrinsics_from_v_(intrinsics_from),
          intrinsics_to_K_only_v_(intrinsics_to_K_only),
          intrinsics_from_J_(kParametersIntrinsics),
          intrinsics_to_K_only_J_(kParametersIntrinsics),
          image_point_(image_point) {
        CHECK(intrinsics_to_K_only.size() == kParametersIntrinsics);
        CHECK(intrinsics_to_K_only.size() == intrinsics_from.size());

        for (std::size_t i = 0; i < kParametersIntrinsics; ++i) {
            intrinsics_from_J_[i] = Jet(intrinsics_from_v_[i]);
            intrinsics_to_K_only_J_[i] = Jet(intrinsics_to_K_only_v_[i]);
        }
    }

    static ceres::CostFunction* Create(
        std::vector<double> const& intrinsics_from,
        std::vector<double> const& intrinsics_to_K_only,
        Eigen::Vector2d const& image_point) {
        return new ceres::AutoDiffCostFunction<PinholeUndistortCost<FL, SK, RA, TA, TP>, 2, 2>(
            new PinholeUndistortCost<FL, SK, RA, TA, TP>(intrinsics_from, intrinsics_to_K_only, image_point));
    }

    template <typename T>
    bool operator()(T const* const undistorted, T* residuals) const {
        T p_u;
        T p_v;
        UndistortedImageToDistortedImage(undistorted[0], undistorted[1], &p_u, &p_v);

        residuals[0] = p_u - T(image_point_[0]);
        residuals[1] = p_v - T(image_point_[1]);

        return true;
    }

private:
    void UndistortedImageToDistortedImage(double const& pu_u, double const& pu_v, double* pd_u, double* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_to_K_only_v_.data(), intrinsics_from_v_.data(), pu_u, pu_v, pd_u, pd_v);
    }

    void UndistortedImageToDistortedImage(Jet const& pu_u, Jet const& pu_v, Jet* pd_u, Jet* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_to_K_only_J_.data(), intrinsics_from_J_.data(), pu_u, pu_v, pd_u, pd_v);
    }

    std::vector<double> intrinsics_from_v_;
    std::vector<double> intrinsics_to_K_only_v_;
    // These are constants and the size doesn't change depending on the camera model.
    // As an optimization we pre-allocate everything instead of the typical "casting"
    // to type T (double to Jet constant constructor).
    std::vector<Jet> intrinsics_from_J_;
    std::vector<Jet> intrinsics_to_K_only_J_;
    Eigen::Vector2d image_point_;
};

//
// Find an undistorted u point by minimizing || d - Distort(u) ||^2 over the undistorted point and the intrinsics
//
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeUndistortWithIntrinsicsCost {
public:
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

private:
    using Jet = ceres::Jet<double, 2 + kParametersIntrinsics>;

public:
    PinholeUndistortWithIntrinsicsCost(
        std::vector<double> const& intrinsics_to_K_only, Eigen::Vector2d const& image_point)
        : intrinsics_to_K_only_v_(intrinsics_to_K_only),
          intrinsics_to_K_only_J_(kParametersIntrinsics),
          image_point_(image_point) {
        CHECK(intrinsics_to_K_only.size() == kParametersIntrinsics);

        for (std::size_t i = 0; i < kParametersIntrinsics; ++i) {
            intrinsics_to_K_only_J_[i] = Jet(intrinsics_to_K_only_v_[i]);
        }
    }

    static ceres::CostFunction* Create(
        std::vector<double> const& intrinsics_to_K_only, Eigen::Vector2d const& image_point) {
        return new ceres::
            AutoDiffCostFunction<PinholeUndistortWithIntrinsicsCost<FL, SK, RA, TA, TP>, 2, kParametersIntrinsics, 2>(
                new PinholeUndistortWithIntrinsicsCost<FL, SK, RA, TA, TP>(intrinsics_to_K_only, image_point));
    }

    template <typename T>
    bool operator()(T const* const intrinsics, T const* const undistorted, T* residuals) const {
        T p_u;
        T p_v;
        UndistortedImageToDistortedImage(intrinsics, undistorted[0], undistorted[1], &p_u, &p_v);

        residuals[0] = p_u - T(image_point_[0]);
        residuals[1] = p_v - T(image_point_[1]);

        return true;
    }

private:
    void UndistortedImageToDistortedImage(
        double const* const intrinsics, double const& pu_u, double const& pu_v, double* pd_u, double* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_to_K_only_v_.data(), intrinsics, pu_u, pu_v, pd_u, pd_v);
    }

    void UndistortedImageToDistortedImage(
        Jet const* const intrinsics, Jet const& pu_u, Jet const& pu_v, Jet* pd_u, Jet* pd_v) const {
        detail::UndistortedImageToDistortedImage<FL, SK, RA, TA, TP>(
            intrinsics_to_K_only_J_.data(), intrinsics, pu_u, pu_v, pd_u, pd_v);
    }

    std::vector<double> intrinsics_to_K_only_v_;
    // These are constants and the size doesn't change depending on the camera model.
    // As an optimization we pre-allocate everything instead of the typical "casting"
    // to type T (double to Jet constant constructor).
    std::vector<Jet> intrinsics_to_K_only_J_;
    Eigen::Vector2d image_point_;
};

}  // namespace nie

#endif  // NIE_CV_CERES_UNDISTORT_COST_HPP
