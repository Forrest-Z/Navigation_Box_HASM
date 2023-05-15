/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by pedro.raimundo on 22/02/19.
//
#ifndef POLYGON_OVERLAP_HPP
#define POLYGON_OVERLAP_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <opencv2/core.hpp>

namespace detail {
template <typename T>
boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<T>> CvPointsToBoostPolygon(
    std::vector<cv::Point_<T>> const& points) {
    using Point = boost::geometry::model::d2::point_xy<T>;
    using Polygon = boost::geometry::model::polygon<Point>;

    std::vector<Point> transformed_points;
    transformed_points.reserve(points.size());

    for (cv::Point_<T> const& point : points) {
        transformed_points.push_back({point.x, point.y});
    }

    Polygon polygon;
    boost::geometry::assign_points(polygon, transformed_points);
    boost::geometry::correct(polygon);

    return polygon;
}

template <typename T>
double GetPolygonOverlapRatio(T const& p0, T const& p1) {
    std::deque<T> intersection;
    boost::geometry::intersection(p0, p1, intersection);
    double overlap_area = 0.0;
    for (auto const& pol : intersection) {
        overlap_area += boost::geometry::area(pol);
    }
    return overlap_area / boost::geometry::area(p0);
}

}  // namespace detail

template <typename T>
double GetPolygonOverlapRatio(std::vector<cv::Point_<T>> const& p0, std::vector<cv::Point_<T>> const& p1) {
    auto boost_p0 = detail::CvPointsToBoostPolygon(p0);
    auto boost_p1 = detail::CvPointsToBoostPolygon(p1);
    return detail::GetPolygonOverlapRatio(boost_p0, boost_p1);
}

#endif  // POLYGON_OVERLAP_HPP
