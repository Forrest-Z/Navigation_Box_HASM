/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef PINHOLE_DISTORT_WEIGHTED_COST_HPP
#define PINHOLE_DISTORT_WEIGHTED_COST_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"

namespace nie {

//
// Find a distorted point d by minimizing || d - Distort(u) ||^2
// This version of distort weighs the residuals by their respective standard deviations.
//
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeDistortWeightedCost {
public:
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

private:
    using Jet = ceres::Jet<double, 2 + kParametersIntrinsics>;

public:
    PinholeDistortWeightedCost(
        std::vector<double> const& intrinsics_from_K_only,
        std::vector<double> const& intrinsics_to,
        std::vector<double> const& intrinsics_to_weights,
        Eigen::Vector2d const& image_point_undistorted)
        : intrinsics_from_K_only_v_(intrinsics_from_K_only),
          intrinsics_to_v_(intrinsics_to),
          intrinsics_to_w_(intrinsics_to_weights),
          intrinsics_from_K_only_J_(intrinsics_from_K_only.size()),
          intrinsics_to_J_(intrinsics_from_K_only.size()),
          image_point_undistorted_(image_point_undistorted) {
        CHECK(intrinsics_from_K_only.size() == intrinsics_to.size());

        for (std::size_t i = 0; i < intrinsics_from_K_only.size(); ++i) {
            intrinsics_from_K_only_J_[i] = Jet(intrinsics_from_K_only_v_[i]);
            intrinsics_to_J_[i] = Jet(intrinsics_to_v_[i]);
        }
    }

    static ceres::CostFunction* Create(
        std::vector<double> const& intrinsics_from_K_only,
        std::vector<double> const& intrinsics_to,
        std::vector<double> const& intrinsics_to_stddevs,
        Eigen::Vector2d const& image_point_undistorted) {
        return new ceres::AutoDiffCostFunction<
            PinholeDistortWeightedCost<FL, SK, RA, TA, TP>,
            2 + kParametersIntrinsics,
            kParametersIntrinsics,
            2>(new PinholeDistortWeightedCost<FL, SK, RA, TA, TP>(
            intrinsics_from_K_only, intrinsics_to, intrinsics_to_stddevs, image_point_undistorted));
    }

    template <typename T>
    bool operator()(T const* const intrinsics, T const* const distorted, T* residuals) const {
        T v_u = T(image_point_undistorted_[0]);
        T v_v = T(image_point_undistorted_[1]);

        ToVector(&v_u, &v_v);

        T p_u;
        T p_v;
        detail::Distort<FL, SK, RA, TA, TP>(intrinsics, v_u, v_v, &p_u, &p_v);

        // Need to re-weight this
        residuals[0] = distorted[0] - p_u;
        residuals[1] = distorted[1] - p_v;
        CalculateIntrinsicsResiduals(intrinsics, residuals);

        for (std::size_t i = 0; i < kParametersIntrinsics; ++i)
            residuals[2 + i] = residuals[2 + i] * intrinsics_to_w_[i];

        return true;
    }

private:
    void ToVector(double* v_u, double* v_v) const {
        detail::BehaviorFocalLength<FL>::Undo(intrinsics_from_K_only_v_.data(), *v_u, *v_v, v_u, v_v);
        detail::BehaviorSkew<SK>::Undo(intrinsics_from_K_only_v_.data(), *v_u, *v_v, v_u, v_v);
    }

    void ToVector(Jet* v_u, Jet* v_v) const {
        detail::BehaviorFocalLength<FL>::Undo(intrinsics_from_K_only_J_.data(), *v_u, *v_v, v_u, v_v);
        detail::BehaviorSkew<SK>::Undo(intrinsics_from_K_only_J_.data(), *v_u, *v_v, v_u, v_v);
    }

    void CalculateIntrinsicsResiduals(double const* const intrinsics, double* residuals) const {
        for (std::size_t i = 0; i < kParametersIntrinsics; ++i) residuals[2 + i] = intrinsics[i] - intrinsics_to_v_[i];
    }

    void CalculateIntrinsicsResiduals(Jet const* const intrinsics, Jet* residuals) const {
        for (std::size_t i = 0; i < kParametersIntrinsics; ++i) residuals[2 + i] = intrinsics[i] - intrinsics_to_J_[i];
    }

    std::vector<double> intrinsics_from_K_only_v_;
    std::vector<double> intrinsics_to_v_;
    std::vector<double> intrinsics_to_w_;
    // These are constants and the size doesn't change depending on the camera model.
    // As an optimization we pre-allocate everything instead of the typical "casting"
    // to type T (double to Jet constant constructor).
    std::vector<Jet> intrinsics_from_K_only_J_;
    std::vector<Jet> intrinsics_to_J_;

    Eigen::Vector2d image_point_undistorted_;
};

}  // namespace nie

#endif  // PINHOLE_DISTORT_WEIGHTED_COST_HPP
