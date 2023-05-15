/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <nie/core/scoped_timer.hpp>

#include <cassert>

namespace nie {

template <typename Isometry>
inline void CalculateBackProjectionInv(
    Eigen::Vector3d const& object,
    std::vector<Isometry> const& poses_cw,
    Eigen::Matrix3d const& K,
    KeypointVector* p_features) {
    ScopedTimer timer("back_projection.inl::CalculateBackProjectionInv");
    assert(p_features != nullptr);

    p_features->clear();
    p_features->reserve(poses_cw.size());
    for (Isometry const& pose_cw : poses_cw) {
        Eigen::Vector2f const p = CalculateBackProjectionInv(object, pose_cw, K);
        p_features->emplace_back(p[0], p[1]);
    }
}

// TODO(MvB): To improve efficiency, check
// https://eigen.tuxfamily.org/dox/group__Geometry__Module.html#gaf99305a3d7432318236df7b80022df37
template <typename Derived, typename Rotation>
inline void CalculateBackProjection(
    std::vector<Eigen::Vector3d> const& objects,
    Isometry3Base<Derived, Rotation> const& motion,
    Eigen::Matrix3d const& K,
    KeypointVector* p_features) {
    ScopedTimer timer("back_projection.inl::CalculateBackProjection");
    assert(p_features != nullptr);

    p_features->reserve(objects.size());

    // Inverse, so world to camera, in matrix form for efficient calculation
    Isometry3md const motion_inv(motion.Inversed());

    for (Eigen::Vector3d const& object : objects) {
        Eigen::Vector2f const p = CalculateBackProjectionInv(object, motion_inv, K);
        p_features->emplace_back(p[0], p[1]);
    }
}

}  // namespace nie
