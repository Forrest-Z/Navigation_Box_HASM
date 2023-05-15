/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_CALIBRATION_PROBLEM_HPP
#define NIE_CV_CERES_CALIBRATION_PROBLEM_HPP

#include <memory>

#include <ceres/ceres.h>

#include "nie/cv/calib3d/distortion_model_parameters.hpp"

namespace nie {

class CalibrationCostDefinition {
public:
    virtual ~CalibrationCostDefinition() = default;

    virtual int GetParameterCountIntrinsics() const = 0;
    virtual std::vector<double> InitializeIntrinsicsFromK(const Eigen::Matrix3d& K) const = 0;
    virtual ceres::CostFunction* Create(
        Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const = 0;
    virtual ceres::CostFunction* CreateWithBaseline(
        Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const = 0;
};

class CalibrationCostDefinitionFactory {
public:
    static std::unique_ptr<CalibrationCostDefinition> Create(const DistortionModelParameters& parameters);
};

}  // namespace nie

#endif  // NIE_CV_CERES_CALIBRATION_PROBLEM_HPP
