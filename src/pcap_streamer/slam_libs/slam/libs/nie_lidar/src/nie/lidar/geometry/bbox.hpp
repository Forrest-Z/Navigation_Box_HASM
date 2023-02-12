/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <array>

#include <pcl/point_cloud.h>

#include "nie/core/geometry/isometry3.hpp"

#include "nie/lidar/helper_point_types.hpp"

namespace nie {

template <typename Scalar>
class Bbox {
public:
    static_assert(
            std::is_floating_point_v<Scalar> || std::is_integral_v<Scalar>, "ONLY FLOAT OR INT TYPES ARE ALLOWED");

    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    template <typename PointT>
    [[nodiscard]] inline static Bbox Create(pcl::PointCloud<PointT> const& point_cloud) {
        nie::Bbox bbox = Bbox::InverseMaxBoundingBox();
        for (auto const& p : point_cloud.points) {
            bbox.FitPoint(p);
        }
        return bbox;
    }

    // Isometry is given as the transformation of a point in the new bounding box to the point in the world
    template <typename PointT>
    [[nodiscard]] inline static Bbox Create(
            pcl::PointCloud<PointT> const& point_cloud, nie::Isometry3qd const& origin) {
        Bbox bbox = Bbox::InverseMaxBoundingBox();
        nie::Isometry3qf const origin_inv = origin.Inversed().cast<float>();
        for (auto const& p : point_cloud.points) {
            Vector3 b;
            b = origin_inv.Transform(p.template getVector3fMap());
            bbox.FitPoint(b);
        }
        return bbox;
    }

    [[nodiscard]] static Bbox MaxBoundingBox() {
        return {{std::numeric_limits<Scalar>::lowest(),
                 std::numeric_limits<Scalar>::lowest(),
                 std::numeric_limits<Scalar>::lowest()},
                {std::numeric_limits<Scalar>::max(),
                 std::numeric_limits<Scalar>::max(),
                 std::numeric_limits<Scalar>::max()}};
    }

    [[nodiscard]] static Bbox InverseMaxBoundingBox() {
        return {{std::numeric_limits<Scalar>::max(),
                 std::numeric_limits<Scalar>::max(),
                 std::numeric_limits<Scalar>::max()},
                {std::numeric_limits<Scalar>::lowest(),
                 std::numeric_limits<Scalar>::lowest(),
                 std::numeric_limits<Scalar>::lowest()}};
    }

    inline void Inflate(Scalar x, Scalar y, Scalar z) {
        min -= Vector3(x, y, z);
        max += Vector3(x, y, z);
    }

    inline void Inflate(Vector3 const& delta) {
        min -= delta;
        max += delta;
    }

    inline void Inflate(Scalar v) { Inflate(v, v, v); }

    // Changes the size of the box to make it fit point
    template <typename PointT, std::enable_if_t<!std::is_base_of_v<Eigen::MatrixBase<PointT>, PointT>, int> = 0>
    inline void FitPoint(PointT const& p) {
        FitPoint(p.template getVector3fMap());
    }

    // Changes the size of the box to make it fit point
    template <typename Derived>
    inline void FitPoint(Eigen::MatrixBase<Derived> const& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    [[nodiscard]] inline Scalar Volume() const { return Range().prod(); }

    [[nodiscard]] inline Vector3 Range() const { return max - min; }

    [[nodiscard]] inline Vector3 Center() const { return Range() * Scalar{0.5} + min; }

    template <typename PointT, std::enable_if_t<!std::is_base_of_v<Eigen::MatrixBase<PointT>, PointT>, int> = 0>
    [[nodiscard]] inline bool Contains(PointT const& p) const {
        return Contains(p.template getVector3fMap());
    }

    template <typename Derived>
    [[nodiscard]] inline bool Contains(Eigen::MatrixBase<Derived> const& p) const {
        return min.x() <= p.x() && p.x() <= max.x() && min.y() <= p.y() && p.y() <= max.y() && min.z() <= p.z() &&
               p.z() <= max.z();
    }

    [[nodiscard]] std::array<Vector3, 8> GetAllCorners() const {
        return {min,
                {min.x(), min.y(), max.z()},
                {min.x(), max.y(), min.z()},
                {max.x(), min.y(), min.z()},
                {min.x(), max.y(), max.z()},
                {max.x(), min.y(), max.z()},
                {max.x(), max.y(), min.z()},
                max};
    }

    Vector3 min;
    Vector3 max;
};

using Bboxf = Bbox<float>;
using Bboxd = Bbox<double>;
using Bboxi = Bbox<int>;

template <typename Scalar>
inline Bbox<Scalar>& operator&=(Bbox<Scalar>& l, Bbox<Scalar> const& r) {
    l.min = l.min.cwiseMax(r.min);
    l.max = l.max.cwiseMin(r.max);
    return l;
}

template <typename Scalar>
// Can have a negative volume.
inline Bbox<Scalar> operator&(Bbox<Scalar> const& l, Bbox<Scalar> const& r) {
    Bbox and_box = l;
    return and_box &= r;
}

template <typename Scalar>
inline Bbox<Scalar>& operator|=(Bbox<Scalar>& l, Bbox<Scalar> const& r) {
    l.min = l.min.cwiseMin(r.min);
    l.max = l.max.cwiseMax(r.max);
    return l;
}

// Can have a negative volume.
template <typename Scalar>
inline Bbox<Scalar> operator|(Bbox<Scalar> const& l, Bbox<Scalar> const& r) {
    Bbox or_box = l;
    return or_box |= r;
}

template <typename Scalar>
inline Bbox<Scalar>& operator+=(Bbox<Scalar>& l, pcl::PointXYZ const& r) {
    l.min += r.getVector3fMap();
    l.max += r.getVector3fMap();
    return l;
}

template <typename Scalar>
inline Bbox<Scalar> operator+(Bbox<Scalar> const& l, pcl::PointXYZ const& r) {
    Bbox add_box = l;
    return add_box += r;
}

template <typename Derived>
inline Bbox<typename Derived::Scalar>& operator-=(
        Bbox<typename Derived::Scalar>& l, Eigen::MatrixBase<Derived> const& r) {
    l.min -= r;
    l.max -= r;
    return l;
}

template <typename Derived>
inline Bbox<typename Derived::Scalar> operator-(
        Bbox<typename Derived::Scalar> const& l, Eigen::MatrixBase<Derived> const& r) {
    Bbox min_box = l;
    return min_box -= r;
}

template <typename Derived>
inline Bbox<typename Derived::Scalar>& operator+=(
        Bbox<typename Derived::Scalar>& l, Eigen::MatrixBase<Derived> const& r) {
    l.min += r;
    l.max += r;
    return l;
}

template <typename Derived>
inline Bbox<typename Derived::Scalar> operator+(
        Bbox<typename Derived::Scalar> const& l, Eigen::MatrixBase<Derived> const& r) {
    Bbox max_box = l;
    return max_box += r;
}

template <typename Scalar>
inline Bbox<Scalar>& operator-=(Bbox<Scalar>& l, pcl::PointXYZ const& r) {
    l -= r.getVector3fMap();
    return l;
}

template <typename Scalar>
inline Bbox<Scalar> operator-(Bbox<Scalar> const& l, pcl::PointXYZ const& r) {
    Bbox min_box = l;
    return min_box -= r;
}

}  // namespace nie

template <typename Scalar>
inline std::ostream& operator<<(std::ostream& os, nie::Bbox<Scalar> const& box) {
    os << "[" << box.min.transpose() << "] [" << box.max.transpose() << "]";
    return os;
}
