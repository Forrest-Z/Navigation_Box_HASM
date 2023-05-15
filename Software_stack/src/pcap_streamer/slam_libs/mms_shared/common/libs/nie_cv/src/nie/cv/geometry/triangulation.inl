/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "line.hpp"

namespace nie {

namespace detail {

/// @brief A mapper to the translation part of the Isometry3 class.
template <typename Isometry>
class Isometry3Origins {
public:
    explicit Isometry3Origins(std::vector<Isometry> const& extrinsics_wc) : extrinsics_wc_(extrinsics_wc) {}

    inline Eigen::Vector3d const& operator[](std::size_t i) const { return extrinsics_wc_[i].translation(); }

    inline std::size_t size() const { return extrinsics_wc_.size(); }

private:
    std::vector<Isometry> const& extrinsics_wc_;
};

/// @brief A mapper from 2d image points to it's corresponding 3d world vectors.
template <typename Isometry>
class KeyPointVectors {
public:
    explicit KeyPointVectors(
        std::vector<Isometry> const& extrinsics_wc,
        std::vector<Eigen::Vector2f> const& key_points,
        Eigen::Matrix3d const& K)
        : extrinsics_wc_(extrinsics_wc), key_points_(key_points), K_inv_(K.inverse()) {
        assert(extrinsics_wc_.size() == key_points_.size());
    }

    inline Eigen::Vector3d operator[](std::size_t i) const {
        return extrinsics_wc_[i].rotation() * K_inv_ * key_points_[i].template cast<double>().homogeneous();
    }

    inline std::size_t size() const { return extrinsics_wc_.size(); }

private:
    std::vector<Isometry> const& extrinsics_wc_;
    std::vector<Eigen::Vector2f> const& key_points_;
    Eigen::Matrix3d const K_inv_;
};

}  // namespace detail

template <typename Isometry>
bool TriangulateLinear(
    std::vector<Isometry> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& key_points,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x) {
    return LinesIntersect(
        detail::Isometry3Origins<Isometry>{extrinsics_wc},
        detail::KeyPointVectors<Isometry>{extrinsics_wc, key_points, K},
        x);
}

}  // namespace nie