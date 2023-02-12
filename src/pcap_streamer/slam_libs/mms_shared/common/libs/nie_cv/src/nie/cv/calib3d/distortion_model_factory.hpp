/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_DISTORTION_MODEL_FACTORY_HPP
#define NIE_CV_CALIB3D_DISTORTION_MODEL_FACTORY_HPP

#include <memory>

#include <Eigen/Core>

#include <nie/core/glog.hpp>

#include "distortion_model.hpp"
#include "distortion_model_parameters.hpp"
#include "pinhole_distortion_model.hpp"

namespace nie {

namespace {

template <typename PinholeModel>
class PinholeDistortionModelWrapper final : public DistortionModel {
public:
    PinholeDistortionModelWrapper(PinholeModel&& model) : model_(std::move(model)) {}

    void Distort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const { model_.Distort(p_u, p_d); }

    void Distort(Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const {
        model_.Distort(K_undistorted, p_u, p_d);
    }

    void Distort(Eigen::Matrix3d const& K_undistorted, Eigen::Vector2d const& p_u, Eigen::Vector2d* p_d) const {
        model_.Distort(K_undistorted, p_u, p_d);
    }

    void Undistort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const { model_.Undistort(p_u, p_d); }

    void Undistort(Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const {
        model_.Undistort(K_undistorted, p_u, p_d);
    }

    void Undistort(
        Eigen::Matrix3d const& K_undistorted,
        std::vector<Eigen::Vector2f> const& p_u,
        std::vector<Eigen::Vector2f>* p_d) const {
        model_.Undistort(K_undistorted, p_u, p_d);
    }

    Eigen::Matrix3d K() const { return model_.K(); }

    const std::vector<double>& intrinsics() const { return model_.intrinsics(); }

    std::vector<double>& intrinsics() { return model_.intrinsics(); };

    DistortionModelParameters parameters() const {
        return {PinholeModel::kParameterFocalLength,
                PinholeModel::kParameterSkew,
                PinholeModel::kParameterDistortionRadial,
                PinholeModel::kParameterDistortionTangential,
                PinholeModel::kParameterDistortionThinPrism};
    }

private:
    PinholeModel model_;
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP,
    typename... Types>
std::unique_ptr<DistortionModel> CreateFromDistortionThinPrism(const DistortionModelParameters&, Types... arguments) {
    PinholeDistortionModel<FL, SK, RA, TA, TP> model(arguments...);

    return std::make_unique<PinholeDistortionModelWrapper<PinholeDistortionModel<FL, SK, RA, TA, TP>>>(
        std::move(model));
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    typename... Types>
std::unique_ptr<DistortionModel> CreateFromDistortionTangential(
    const DistortionModelParameters& parameters, Types... arguments) {
    switch (parameters.distortion_thin_prism) {
        case ParameterDistortionThinPrism::S4:
            return CreateFromDistortionThinPrism<FL, SK, RA, TA, ParameterDistortionThinPrism::S4>(
                parameters, arguments...);
        case ParameterDistortionThinPrism::NO_DISTORTION:
            return CreateFromDistortionThinPrism<FL, SK, RA, TA, ParameterDistortionThinPrism::NO_DISTORTION>(
                parameters, arguments...);
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionThinPrism";
    }
}

template <ParameterFocalLength FL, ParameterSkew SK, ParameterDistortionRadial RA, typename... Types>
std::unique_ptr<DistortionModel> CreateFromDistortionRadial(
    const DistortionModelParameters& parameters, Types... arguments) {
    switch (parameters.distortion_tangential) {
        case ParameterDistortionTangential::P2:
            return CreateFromDistortionTangential<FL, SK, RA, ParameterDistortionTangential::P2>(
                parameters, arguments...);
        case ParameterDistortionTangential::NO_DISTORTION:
            return CreateFromDistortionTangential<FL, SK, RA, ParameterDistortionTangential::NO_DISTORTION>(
                parameters, arguments...);
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionTangential";
    }
}

template <ParameterFocalLength FL, ParameterSkew SK, typename... Types>
std::unique_ptr<DistortionModel> CreateFromSkew(const DistortionModelParameters& parameters, Types... arguments) {
    switch (parameters.distortion_radial) {
        case ParameterDistortionRadial::K3:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::K3>(parameters, arguments...);
        case ParameterDistortionRadial::K6:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::K6>(parameters, arguments...);
        case ParameterDistortionRadial::NO_DISTORTION:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::NO_DISTORTION>(
                parameters, arguments...);
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterDistortionRadial";
    }
}

template <ParameterFocalLength FL, typename... Types>
std::unique_ptr<DistortionModel> CreateFromFocalLength(
    const DistortionModelParameters& parameters, Types... arguments) {
    switch (parameters.skew) {
        case ParameterSkew::SKEW:
            return CreateFromSkew<FL, ParameterSkew::SKEW>(parameters, arguments...);
        case ParameterSkew::NO_SKEW:
            return CreateFromSkew<FL, ParameterSkew::NO_SKEW>(parameters, arguments...);
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterSkew";
    }
}

}  // namespace

class DistortionModelFactory {
public:
    template <typename... Types>
    static std::unique_ptr<DistortionModel> Create(const DistortionModelParameters& parameters, Types... arguments);
};

template <typename... Types>
std::unique_ptr<DistortionModel> DistortionModelFactory::Create(
    const DistortionModelParameters& parameters, Types... arguments) {
    switch (parameters.focal_length) {
        case ParameterFocalLength::XY_F:
            return CreateFromFocalLength<ParameterFocalLength::XY_F>(parameters, arguments...);
        case ParameterFocalLength::SINGLE_F:
            return CreateFromFocalLength<ParameterFocalLength::SINGLE_F>(parameters, arguments...);
        default:
            LOG(FATAL) << "Unknown enum nie::ParameterFocalLength";
    }
}

}  // namespace nie

#endif  // NIE_CV_CALIB3D_DISTORTION_MODEL_FACTORY_HPP
