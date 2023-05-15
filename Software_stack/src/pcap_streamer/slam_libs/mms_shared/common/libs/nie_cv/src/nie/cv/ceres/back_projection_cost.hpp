/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_BACK_PROJECTION_COST_HPP
#define NIE_CV_CERES_BACK_PROJECTION_COST_HPP

#include <Eigen/Core>

#include "helper_ceres.hpp"
#include "nie/cv/calib3d/pinhole_distortion_model.hpp"

namespace nie {

// TODO(jbr): Old intrinsics adapter code below should be templated in the back projection cost as a choice.
// TODO(jbr): This can be changed to work in a similar manner as the CalibrationCost.
// TODO(jbr): Perhaps in the future when we want to include using the skew within triangulation or bundle adjustment.
namespace detail {

// TODO(jbr): Perhaps move this somewhere nice.
template <typename T>
std::vector<T> CreateCastedVector(std::vector<double> const& in) {
    std::vector<T> out;
    out.reserve(in.size());
    std::transform(in.begin(), in.end(), std::back_inserter(out), [](double const& v) -> T { return T(v); });
    return out;
}

template <ParameterFocalLength FL, ParameterSkew SK>
class BaseBackProjectionCost {
public:
    static constexpr int kParametersTranslation = detail::kParamatersTranslation;
    static constexpr int kParametersRotation = detail::kParametersRotation;
    static constexpr int kParametersObjectPoint = detail::kParametersObjectPoint;

    BaseBackProjectionCost(std::vector<double> intrinsics, Eigen::Vector2f const& image_point)
        : intrinsics_(std::move(intrinsics)), image_point_(image_point.cast<double>()) {}

protected:
    template <typename Isometry, typename T>
    inline void CalculateBackProjectionError(
        Isometry const& extrinsics, const T* const object_point, T* residuals) const {
        // TODO(jbr): We may optimize this later if we know what Jet type is used.
        std::vector<T> intrinsics = detail::CreateCastedVector<T>(intrinsics_);
        Eigen::Matrix<T, 2, 1> image_point = image_point_.cast<T>();

        detail::CalculateBackProjectionError<FL, SK>(
            extrinsics, intrinsics.data(), object_point, image_point.template data(), residuals);
    }

    std::vector<double> intrinsics_;
    Eigen::Vector2d image_point_;
};

}  // namespace detail

// Old intrinsics adapter
// Since the user can't chose the model yet. This is a convenience function.
inline static std::vector<double> CreateIntrinsicsVector(Eigen::Matrix3d const& K) {
    return nie::PinholeDistortionModel<
        nie::ParameterFocalLength::XY_F,
        nie::ParameterSkew::NO_SKEW,
        nie::ParameterDistortionRadial::NO_DISTORTION,
        nie::ParameterDistortionTangential::NO_DISTORTION,
        nie::ParameterDistortionThinPrism::NO_DISTORTION>::CreateVector(K);
}

/// @brief Minimizes over the object points.
class BackprojectionCost final
    : public detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW> {
private:
    using Base = detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW>;

public:
    BackprojectionCost(std::vector<double> intrinsics, Eigen::Vector2f const& image_point, nie::Isometry3qd extrinsics)
        : Base(std::move(intrinsics), image_point), extrinsics_(std::move(extrinsics)) {}

    static ceres::CostFunction* Create(
        std::vector<double> intrinsics, Eigen::Vector2f const& image_point, nie::Isometry3qd extrinsics) {
        return new ceres::AutoDiffCostFunction<BackprojectionCost, 2, kParametersObjectPoint>(
            new BackprojectionCost(std::move(intrinsics), image_point, std::move(extrinsics)));
    }

    template <typename T>
    bool operator()(const T* const object_point, T* residuals) const {
        auto extrinsics = extrinsics_.cast<T>();
        CalculateBackProjectionError(extrinsics, object_point, residuals);
        return true;
    }

private:
    nie::Isometry3qd extrinsics_;
};

/// @brief Minimizes over the extrinsics and object points.
class BackprojectionWithExtrinsicsCost final
    : public detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW> {
private:
    using Base = detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW>;

public:
    BackprojectionWithExtrinsicsCost(std::vector<double> intrinsics, Eigen::Vector2f const& image_point)
        : Base(std::move(intrinsics), image_point) {}

    static ceres::CostFunction* Create(std::vector<double> intrinsics, Eigen::Vector2f const& image_point) {
        return new ceres::AutoDiffCostFunction<
            BackprojectionWithExtrinsicsCost,
            2,
            kParametersTranslation,
            kParametersRotation,
            kParametersObjectPoint>(new BackprojectionWithExtrinsicsCost(std::move(intrinsics), image_point));
    }

    template <typename T>
    bool operator()(
        const T* const translation, const T* const rotation, const T* const object_point, T* residuals) const {
        Isometry3Map<const Eigen::Quaternion<T>> extrinsics{translation, rotation};
        CalculateBackProjectionError(extrinsics, object_point, residuals);
        return true;
    }
};

/// @brief Minimizes over the extrinsics and object points. Uses the baseline as a calibration constant to change camera
/// frame.
class BackprojectionWithBaselineCost final
    : public detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW> {
private:
    using Base = detail::BaseBackProjectionCost<nie::ParameterFocalLength::XY_F, nie::ParameterSkew::NO_SKEW>;

public:
    BackprojectionWithBaselineCost(
        std::vector<double> intrinsics, Eigen::Vector2f const& image_point, Isometry3qd baseline)
        : Base(std::move(intrinsics), image_point), baseline_(std::move(baseline)) {}

    static ceres::CostFunction* Create(
        std::vector<double> intrinsics, Eigen::Vector2f const& image_point, Isometry3qd baseline) {
        return new ceres::AutoDiffCostFunction<
            BackprojectionWithBaselineCost,
            2,
            kParametersTranslation,
            kParametersRotation,
            kParametersObjectPoint>(
            new BackprojectionWithBaselineCost(std::move(intrinsics), image_point, std::move(baseline)));
    }

    template <typename T>
    bool operator()(
        const T* const translation, const T* const rotation, const T* const object_point, T* residuals) const {
        auto extrinsics = Isometry3Map<const Eigen::Quaternion<T>>{translation, rotation} * baseline_.cast<T>();
        CalculateBackProjectionError(extrinsics, object_point, residuals);
        return true;
    }

private:
    Isometry3qd baseline_;
};

}  // namespace nie

#endif  // NIE_CV_CERES_BACK_PROJECTION_COST_HPP
