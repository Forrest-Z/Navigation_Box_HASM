/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once


#include <numeric>  // accumulate

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <nie/core/geometry/isometry3.hpp>

#include "bbox.hpp"

namespace nie {

class PoseBbox {
private:
    using Point = boost::geometry::model::d2::point_xy<double>;
    using Polygon = boost::geometry::model::polygon<Point>;

public:
    PoseBbox() : origin_(), bbox_() {}
    PoseBbox(Eigen::Vector3d translation, Bboxf bbox)
        : origin_(Isometry3qd::FromTranslation(std::move(translation))), bbox_(bbox) {}
    PoseBbox(Isometry3qd origin, Bboxf bbox) : origin_(std::move(origin)), bbox_(bbox) {}

    [[nodiscard]] inline PoseBbox CopyWithOrigin(Eigen::Vector3d const& translation) const {
        // Change it into a relative offset to keep precision when updating the float bounds
        auto t_ab_points = origin_.translation() - translation;
        return {translation, bbox_ + t_ab_points.cast<float>()};
    }

    template <typename PointT>
    inline void UpdateBbox(pcl::PointCloud<PointT> const& point_cloud) {
        bbox_ = Bboxf::Create(point_cloud);
    }

    template <typename Derived>
    [[nodiscard]] inline bool Contains(Eigen::MatrixBase<Derived> const& matrix) const {
        if constexpr (std::is_same_v<typename Derived::Scalar, float>) {
            return bbox_.Contains(origin_.TransformInverseLeft(matrix.template cast<double>()).template cast<float>());
        } else {
            return bbox_.Contains(origin_.TransformInverseLeft(matrix).template cast<float>());
        }
    }

    // Gives a reasonable polygon estimate of the ground plane when world coordinates are in the aircraft convention. Result is in world coordinates.
    [[nodiscard]] std::array<Eigen::Vector3d, 4> GroundPlanePoly() const {
        // We don't know anything about the posebox other than that we should be able to 
        // draw it in world coords. "Should" means that the world up needs correspond to
        // a local up in the bounding box unless the vehicle was moving under extreme angles.
        // We roughly determine the bottom plane and return its poly.
        Eigen::Vector3d up = (origin_.rotation().conjugate() * Eigen::Vector3d::UnitZ()).cwiseAbs();
        Eigen::Vector3d::Index index;
        up.maxCoeff(&index);
        
        Eigen::Vector3d min = bbox_.min.cast<double>();
        Eigen::Vector3d max = bbox_.max.cast<double>();
        max(index) = min(index);
        std::array<Eigen::Vector3d, 4> poly {
            min,
            min,
            max,
            max
        };

        Eigen::Vector3d dlt = max - min;
        dlt.maxCoeff(&index);
        poly[1](index) = poly[2](index);
        poly[3](index) = poly[0](index);

        for (auto& p : poly) {
            p = origin_ * p;
        }

        Eigen::Vector3d diag = poly[2] - poly[0];
        Eigen::Vector3d orth = poly[3] - poly[0];

        // if CCW, make it CW: that's what boost wants
        if ((diag(0) * orth(1) - diag(1) * orth(0)) > 0.0) {
            std::swap(poly[1], poly[3]);
        }
        
        return poly;
    }

    /// \brief Calculates the 2D ground plane intersection between two bounds.
    /// \param other Bounds to intersect with.
    /// \param area Output 2D ground plane intersection area.
    /// \param iou Output intersection over union.
    void Intersection2D(PoseBbox const& other, double* area, double* iou) const {
        auto poly_a = GroundPlanePolyBoost();
        auto poly_b = other.GroundPlanePolyBoost();
        auto area_a = boost::geometry::area(poly_a);
        auto area_b = boost::geometry::area(poly_b);

        std::vector<Polygon> intersections;
        boost::geometry::intersection(poly_a, poly_b, intersections);

        *area = std::accumulate(
            intersections.begin(), intersections.end(), 0.0, [](double s, Polygon const& p) -> double {
                return s + boost::geometry::area(p);
            });

        *iou = *area / (area_a + area_b - *area);
    }

    [[nodiscard]] Isometry3qd const& origin() const { return origin_; }
    [[nodiscard]] Isometry3qd& origin() { return origin_; }
    [[nodiscard]] Bboxf const& bbox() const { return bbox_; }
    [[nodiscard]] Bboxf& bbox() { return bbox_; }

private:
    [[nodiscard]] Polygon GroundPlanePolyBoost() const {
        Polygon boost;
        auto eigen = GroundPlanePoly();
        for (auto const& e : eigen) {
            boost::geometry::append(boost.outer(), Point(e.x(), e.y()));
        }
        boost::geometry::append(boost.outer(), boost.outer().front());

        return boost;
    }

    Isometry3qd origin_;
    Bboxf bbox_;
};

}  // namespace nie
