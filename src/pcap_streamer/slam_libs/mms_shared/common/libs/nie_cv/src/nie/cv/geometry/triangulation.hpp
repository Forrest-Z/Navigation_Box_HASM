/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_GEOMETRY_TRIANGULATION_HPP
#define NIE_CV_GEOMETRY_TRIANGULATION_HPP

#include <thread>
#include <vector>

#include <nie/core/geometry/isometry3.hpp>

#include <opencv2/core.hpp>

// TODO(jbr): Perhaps bring back a version that handles variances only.

namespace nie {

// Try to find the 3d point corresponding to a vector of image observations. These
// image observations can be transformed to directional vectors using its respective
// internal camera parameters. Lines from image observations won't generally intersect
// in 3D and thus we try to minimize the sum of distances between the 3d point and all
// lines.
//
// Input:
//  The positions and orientations of each camera plus their corresponding image
//  points. Matrix K equals the internal camera matrix for all camera poses.
// Output:
//  The 3D point.
// Returns:
//  False in case or parallel lines or there aren't a minimum of 2 well defined
//  correspondences.
//
// Note:
//  This method internally calls LinesIntersect(...).
template <typename Isometry>
bool TriangulateLinear(
    std::vector<Isometry> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x);

// Try to find the 3d point corresponding to a vector of image observations. The image
// correspondences won't generally intersect in 3D and thus we try to minimize the sum
// of back projection errors:
//  sum min || u - P(x) || ^ 2 over x, where u equals an image point.
//
// Input:
//  The positions and orientations of each camera plus their corresponding image
//  points. Matrix K equals the internal camera matrix for all camera poses.
// Output:
//  The 3D point.
// Returns:
//  False in case or parallel lines or there aren't a minimum of 2 well defined
//  correspondences.
// TODO (jbr): Slightly more generic triangulate linear interface that accepts an eigen matrix instead of
// TODO (jbr): a list of points (or both).
bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    unsigned int const num_threads = std::thread::hardware_concurrency());

// FIXME(jbr): Deprecated. DO NOT USE. Only backwards compatibility.
// OpenCV version.
[[deprecated]] bool TriangulateNonLinear(
    std::vector<cv::Point3d> const& translations_wc,
    std::vector<cv::Vec3d> const& rotations_wc,
    std::vector<cv::Point2f> const& image_points,
    cv::Matx33d const& K,
    cv::Point3d* x,
    unsigned int const num_threads = std::thread::hardware_concurrency());

// Try to find the 3d point corresponding to a vector of image observations. The image
// correspondences won't generally intersect in 3D and thus we try to minimize the sum
// of back projection errors:
//  sum min || u - P(x) || ^ 2 over x, where u equals an image point.
//
// Input:
//  The positions and orientations of each camera plus their corresponding image
//  points. Matrix K equals the internal camera matrix for all camera poses.
//  Corresponding variances/covariances are required for all inputs.
// Output:
//  The 3D point and its standard deviations.
// Returns:
//  False in case or parallel lines or there aren't a minimum of 2 well defined
//  correspondences.
bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    std::vector<Eigen::Matrix<double, 6, 6>> const& extrinsics_wc_cov,
    std::vector<double> const& image_points_var,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    Eigen::Matrix3d* x_cov,
    unsigned int const num_threads = std::thread::hardware_concurrency());

// Try to find the 3d point corresponding to a vector of image observations. The image
// correspondences won't generally intersect in 3D and thus we try to minimize the sum
// of back projection errors:
//  sum min || u - P(x) || ^ 2 over x, where u equals an image point.
//
// Input:
//  The positions and orientations of each camera plus their corresponding image
//  points. Matrix K equals the internal camera matrix for all camera poses.
//  Corresponding variances/covariances are required for all inputs.
// Output:
//  The 3D point and its standard deviations.
// Returns:
//  False in case or parallel lines or there aren't a minimum of 2 well defined
//  correspondences.
bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    std::vector<Eigen::Matrix<double, 6, 6>> const& extrinsics_wc_cov,
    double const& image_points_var,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    Eigen::Matrix3d* x_cov,
    unsigned int const num_threads = std::thread::hardware_concurrency());

}  // namespace nie

#include "triangulation.inl"

#endif  // NIE_CV_GEOMETRY_TRIANGULATION_HPP
