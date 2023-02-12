/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_DISTORTION_MODEL_HPP
#define NIE_CV_CALIB3D_DISTORTION_MODEL_HPP

#include <Eigen/Core>

#include "nie/cv/calib3d/distortion_model_parameters.hpp"

namespace nie {

class DistortionModel {
public:
    virtual ~DistortionModel() = default;

    virtual void Distort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const = 0;
    virtual void Distort(
        Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const = 0;
    virtual void Undistort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const = 0;
    virtual void Undistort(
        Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const = 0;
    virtual void Undistort(
        Eigen::Matrix3d const& K_undistorted,
        std::vector<Eigen::Vector2f> const& p_u,
        std::vector<Eigen::Vector2f>* p_d) const = 0;

    virtual Eigen::Matrix3d K() const = 0;
    virtual const std::vector<double>& intrinsics() const = 0;
    virtual std::vector<double>& intrinsics() = 0;
    virtual DistortionModelParameters parameters() const = 0;
};

}  // namespace nie

#endif  // NIE_CV_CALIB3D_DISTORTION_MODEL_HPP
