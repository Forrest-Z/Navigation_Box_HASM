/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/core/geometry/isometry3.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

/// Trait for checking if @tparam{T} has an attribute that is obtainable by @tparam{AttrGetter}.
template <typename T, template <typename> class AttrGetter>
struct HasAttr {
    template <typename T_>
    static constexpr bool has(AttrGetter<T_>) {
        return true;
    }
    template <typename>
    static constexpr bool has(...) {
        return false;
    }
    static constexpr bool value = has<T>(0);
};

/// "Attribute Getters" for "x", "y" and "z" attributes.
/// They assume the type of the requested attribute in @tparam{T}.
template <typename T>
using GetAttrTypeX = decltype(std::declval<T>().x);
template <typename T>
using GetAttrTypeY = decltype(std::declval<T>().y);
template <typename T>
using GetAttrTypeZ = decltype(std::declval<T>().z);

/// Check if @tparam{XYZ} has attributes named "x", "y" and "z".
/// If it has these attributes then value is true, else value is false.
template <typename XYZ>
struct HasAttrXyz {
    static constexpr bool value =
        HasAttr<XYZ, GetAttrTypeX>::value && HasAttr<XYZ, GetAttrTypeY>::value && HasAttr<XYZ, GetAttrTypeZ>::value;
};

/// Compute Scalar-type Euclidean distance between two input types.
/// Supported input types:
///     - nie::Isometry3
///     - XYZ type (any struct with attributes .x .y .z)
///     - nie::io::PoseRecord
struct EuclideanDistance {
    /// EuclideanDistance(isometry1, isometry2)
    /// Computes norm of difference between the translations of two Isometry objects: ||isometry1 - isometry2||.
    /// This metric can only applied to two Isometries that are defined in the same coordinate system.
    template <
        typename Derived1,
        typename Derived2,
        typename Rotation1 = typename Derived1::Rotation,
        typename Rotation2 = typename Derived2::Rotation>
    [[nodiscard]] auto operator()(
        nie::Isometry3Base<Derived1, Rotation1> const& isometry1,
        nie::Isometry3Base<Derived2, Rotation2> const& isometry2) const {
        //
        return (isometry1.translation() - isometry2.translation()).norm();
    }

    /// Only for types that have "x", "y" and "z" as attributes.
    template <typename XYZ, typename std::enable_if_t<HasAttrXyz<XYZ>::value, void*> = nullptr>
    [[nodiscard]] auto operator()(XYZ const& p1, XYZ const& p2) const {
        using Scalar = decltype(p1.x);
        using Vector = Eigen::Matrix<Scalar, 3, 1>;
        return (Eigen::Map<Vector const>{&(p1.x)} - Eigen::Map<Vector const>{&(p2.x)}).norm();
    }

    [[nodiscard]] double operator()(
        nie::io::PoseRecord const& pose_record1, nie::io::PoseRecord const& pose_record2) const {
        return operator()(pose_record1.isometry, pose_record2.isometry);
    }
};

}  // namespace nie
