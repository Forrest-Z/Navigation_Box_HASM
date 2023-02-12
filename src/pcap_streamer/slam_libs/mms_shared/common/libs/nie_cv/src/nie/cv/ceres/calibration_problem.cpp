/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "calibration_problem.hpp"

#include "calibration_cost.hpp"
#include "nie/cv/calib3d/pinhole_distortion_model.hpp"

namespace nie {

namespace {

template <typename PinholeModel>
class PinholeCalibrationCostDefinition final : public CalibrationCostDefinition {
public:
    int GetParameterCountIntrinsics() const { return PinholeModel::kParametersIntrinsics; }

    std::vector<double> InitializeIntrinsicsFromK(Eigen::Matrix3d const& K) const {
        return PinholeDistortionModel<
                       PinholeModel::kParameterFocalLength,
                       PinholeModel::kParameterSkew,
                       PinholeModel::kParameterDistortionRadial,
                       PinholeModel::kParameterDistortionTangential,
                       PinholeModel::kParameterDistortionThinPrism>(K)
                .intrinsics();
    }

    ceres::CostFunction* Create(Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const {
        return PinholeCalibrationCost<
                PinholeModel::kParameterFocalLength,
                PinholeModel::kParameterSkew,
                PinholeModel::kParameterDistortionRadial,
                PinholeModel::kParameterDistortionTangential,
                PinholeModel::kParameterDistortionThinPrism>::Create(object_point, image_point);
    }

    ceres::CostFunction* CreateWithBaseline(
            Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const {
        return PinholeWithBaselineCalibrationCost<
                PinholeModel::kParameterFocalLength,
                PinholeModel::kParameterSkew,
                PinholeModel::kParameterDistortionRadial,
                PinholeModel::kParameterDistortionTangential,
                PinholeModel::kParameterDistortionThinPrism>::Create(object_point, image_point);
    }
};

template <
        ParameterFocalLength FL,
        ParameterSkew SK,
        ParameterDistortionRadial RA,
        ParameterDistortionTangential TA,
        ParameterDistortionThinPrism TP>
std::unique_ptr<CalibrationCostDefinition> CreateFromDistortionThinPrism(
        const DistortionModelParameters& /*parameters*/) {
    return std::make_unique<
            PinholeCalibrationCostDefinition<detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>>>();
}

template <ParameterFocalLength FL, ParameterSkew SK, ParameterDistortionRadial RA, ParameterDistortionTangential TA>
std::unique_ptr<CalibrationCostDefinition> CreateFromDistortionTangential(const DistortionModelParameters& parameters) {
    switch (parameters.distortion_thin_prism) {
        case ParameterDistortionThinPrism::S4:
            return CreateFromDistortionThinPrism<FL, SK, RA, TA, ParameterDistortionThinPrism::S4>(parameters);
        case ParameterDistortionThinPrism::NO_DISTORTION:
            return CreateFromDistortionThinPrism<FL, SK, RA, TA, ParameterDistortionThinPrism::NO_DISTORTION>(
                    parameters);
        default:
            LOG(FATAL) << "Unhandled ParameterDistortionThinPrism value.";
    }
}

template <ParameterFocalLength FL, ParameterSkew SK, ParameterDistortionRadial RA>
std::unique_ptr<CalibrationCostDefinition> CreateFromDistortionRadial(const DistortionModelParameters& parameters) {
    switch (parameters.distortion_tangential) {
        case ParameterDistortionTangential::P2:
            return CreateFromDistortionTangential<FL, SK, RA, ParameterDistortionTangential::P2>(parameters);
        case ParameterDistortionTangential::NO_DISTORTION:
            return CreateFromDistortionTangential<FL, SK, RA, ParameterDistortionTangential::NO_DISTORTION>(parameters);
        default:
            LOG(FATAL) << "Unhandled ParameterDistortionTangential value.";
    }
}

template <ParameterFocalLength FL, ParameterSkew SK>
std::unique_ptr<CalibrationCostDefinition> CreateFromSkew(const DistortionModelParameters& parameters) {
    switch (parameters.distortion_radial) {
        case ParameterDistortionRadial::K3:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::K3>(parameters);
        case ParameterDistortionRadial::K6:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::K6>(parameters);
        case ParameterDistortionRadial::NO_DISTORTION:
            return CreateFromDistortionRadial<FL, SK, ParameterDistortionRadial::NO_DISTORTION>(parameters);
        default:
            LOG(FATAL) << "Unhandled ParameterDistortionRadial value.";
    }
}

template <ParameterFocalLength FL>
std::unique_ptr<CalibrationCostDefinition> CreateFromFocalLength(const DistortionModelParameters& parameters) {
    switch (parameters.skew) {
        case ParameterSkew::SKEW:
            return CreateFromSkew<FL, ParameterSkew::SKEW>(parameters);
        case ParameterSkew::NO_SKEW:
            return CreateFromSkew<FL, ParameterSkew::NO_SKEW>(parameters);
        default:
            LOG(FATAL) << "Unhandled ParameterSkew value.";
    }
}

}  // namespace

std::unique_ptr<CalibrationCostDefinition> CalibrationCostDefinitionFactory::Create(
        const DistortionModelParameters& parameters) {
    switch (parameters.focal_length) {
        case ParameterFocalLength::XY_F:
            return CreateFromFocalLength<ParameterFocalLength::XY_F>(parameters);
        case ParameterFocalLength::SINGLE_F:
            return CreateFromFocalLength<ParameterFocalLength::SINGLE_F>(parameters);
        default:
            LOG(FATAL) << "Unhandled ParameterFocalLength value.";
    }
}

}  // namespace nie
