/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef CALIB3D_PINHOLE_DISTORT_WEIGHTED_COST_HPP
#define CALIB3D_PINHOLE_DISTORT_WEIGHTED_COST_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"

namespace nie {

//
// Find an undistorted u point by minimizing || d - Distort(u) ||^2
// This version of undistort weighs the residuals by their respective standard deviations.
//
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeUndistortWeightedCost {
public:
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

private:
    using Jet = ceres::Jet<double, 2 + kParametersIntrinsics>;

public:
    PinholeUndistortWeightedCost(
        std::vector<double> const& intrinsics_from,
        std::vector<double> const& intrinsics_from_weights,
        std::vector<double> const& intrinsics_to_K_only,
        Eigen::Vector2d const& image_point)
        : intrinsics_from_v_(intrinsics_from),
          intrinsics_from_w_(intrinsics_from_weights),
          intrinsics_to_K_only_v_(intrinsics_to_K_only),
          intrinsics_from_J_(intrinsics_from.size()),
          intrinsics_to_K_only_J_(intrinsics_to_K_only.size()),
          image_point_(image_point) {
        CHECK(intrinsics_to_K_only.size() == intrinsics_from.size());

        for (std::size_t i = 0; i < intrinsics_to_K_only_v_.size(); ++i) {
            intrinsics_from_J_[i] = Jet(intrinsics_from_v_[i]);
            intrinsics_to_K_only_J_[i] = Jet(intrinsics_to_K_only_v_[i]);
        }
    }

    static ceres::CostFunction* Create(
        std::vector<double> const& intrinsics_from,
        std::vector<double> const& intrinsics_from_weights,
        std::vector<double> const& intrinsics_to_K_only,
        Eigen::Vector2d const& image_point) {
        return new ceres::AutoDiffCostFunction<
            PinholeUndistortWeightedCost<FL, SK, RA, TA, TP>,
            2 + kParametersIntrinsics,
            kParametersIntrinsics,
            2>(new PinholeUndistortWeightedCost<FL, SK, RA, TA, TP>(
            intrinsics_from, intrinsics_from_weights, intrinsics_to_K_only, image_point));
    }

    template <typename T>
    bool operator()(T const* const intrinsics, T const* const undistorted, T* residuals) const {
        T v_u;
        T v_v;
        ToVector(undistorted, &v_u, &v_v);

        T p_u;
        T p_v;
        detail::Distort<FL, SK, RA, TA, TP, T>(intrinsics, v_u, v_v, &p_u, &p_v);

        residuals[0] = p_u - T(image_point_[0]);
        residuals[1] = p_v - T(image_point_[1]);
        CalculateIntrinsicsResiduals(intrinsics, residuals);

        for (std::size_t i = 0; i < kParametersIntrinsics; ++i)
            residuals[2 + i] = residuals[2 + i] * T(intrinsics_from_w_[i]);

        return true;
    }

private:
    void ToVector(double const* const undistorted, double* v_u, double* v_v) const {
        *v_u = undistorted[0];
        *v_v = undistorted[1];
        detail::BehaviorFocalLength<FL>::Undo(intrinsics_to_K_only_v_.data(), *v_u, *v_v, v_u, v_v);
        detail::BehaviorSkew<SK>::Undo(intrinsics_to_K_only_v_.data(), *v_u, *v_v, v_u, v_v);
    }

    void ToVector(Jet const* const undistorted, Jet* v_u, Jet* v_v) const {
        *v_u = undistorted[0];
        *v_v = undistorted[1];
        detail::BehaviorFocalLength<FL>::Undo(intrinsics_to_K_only_J_.data(), *v_u, *v_v, v_u, v_v);
        detail::BehaviorSkew<SK>::Undo(intrinsics_to_K_only_J_.data(), *v_u, *v_v, v_u, v_v);
    }

    void CalculateIntrinsicsResiduals(double const* const intrinsics, double* residuals) const {
        for (std::size_t i = 0; i < kParametersIntrinsics; ++i)
            residuals[2 + i] = intrinsics[i] - intrinsics_from_v_[i];
    }

    void CalculateIntrinsicsResiduals(Jet const* const intrinsics, Jet* residuals) const {
        for (std::size_t i = 0; i < kParametersIntrinsics; ++i)
            residuals[2 + i] = intrinsics[i] - intrinsics_from_J_[i];
    }

    std::vector<double> intrinsics_from_v_;
    std::vector<double> intrinsics_from_w_;
    std::vector<double> intrinsics_to_K_only_v_;
    // These are constants and the size doesn't change depending on the camera model.
    // As an optimization we pre-allocate everything instead of the typical "casting"
    // to type T (double to Jet constant constructor).
    std::vector<Jet> intrinsics_from_J_;
    std::vector<Jet> intrinsics_to_K_only_J_;
    Eigen::Vector2d image_point_;
};

}  // namespace nie

#endif  // CALIB3D_PINHOLE_DISTORT_WEIGHTED_COST_HPP
