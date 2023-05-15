/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_BACK_PROJECTION_HPP
#define NIE_CV_VO_BACK_PROJECTION_HPP

#include <nie/core/geometry/isometry3.hpp>
#include <opencv2/core/eigen.hpp>

#include "feature/keypoint.hpp"

namespace nie {

/// These are back-projection functions that can be called with "regular" poses, so camera to world. Other functions take
/// already inversed poses (world to camera) for efficient calculation.

/// @brief: Calculate the back-projection of an object seen from a certain inversed pose
/// @param: object 3D object
/// @param: pose_cw already inversed pose, so world to camera
/// @param: K camera matrix
inline Eigen::Vector2f CalculateBackProjectionInv(
    Eigen::Vector3d const& object, Isometry3md const& pose_cw, Eigen::Matrix3d const& K) {
    return (K * (pose_cw * object)).hnormalized().cast<float>();
}

/// @brief: Calculate the back-projections of an object seen from certain inversed poses
/// @param: object 3D object
/// @param: poses_cw already inversed poses, so world to camera
/// @param: K camera matrix
template <typename Isometry>
inline void CalculateBackProjectionInv(
    Eigen::Vector3d const& object,
    std::vector<Isometry> const& poses_cw,
    Eigen::Matrix3d const& K,
    KeypointVector* p_features);

/// @brief: Calculate the back-projections of objects from a certain pose
/// @param: objects 3D objects
/// @param: pose regular pose, camera to world
/// @param: K camera matrix
template <typename Derived, typename Rotation>
inline void CalculateBackProjection(
    std::vector<Eigen::Vector3d> const& objects,
    Isometry3Base<Derived, Rotation> const& pose,
    Eigen::Matrix3d const& K,
    KeypointVector* p_features);

}  // namespace nie

#include "back_projection.inl"

#endif  // NIE_CV_VO_BACK_PROJECTION_HPP
