/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/geometry/rotation.hpp>

#include "nie/lidar/io/lidar_streamer/lidar_data_types.hpp"

namespace nie {

// This filter internally creates an image with its width and height corresponding to a full 360 rotation and the number
// of laser beams, respectively. This image bins any property, like laser distance or x, y or z coordinate. The goal is
// create an image (with one row less) that depicts the angle difference with the naive ground x,y plane for two
// sequential rows in a col.
// Implementation after Efficient Online Segmentation for Sparse 3d Laser Scans by Bogoslavskyi, 2014, section 3.
class GroundPlaneFilter {
private:
    using PointT = pcl::PointXYZI;

public:
    struct Options {
        int columns = 360;

        // The angles on the first/bottom/closest row are marked as ground when they are below the given threshold
        double init_angle = 5. * kDeg2Rad<>;
        double max_normal_angle = 45. * kDeg2Rad<>;  // overall max
        double diff_angle = 10. * kDeg2Rad<>;        // max diff
    };

    GroundPlaneFilter(std::size_t const& num_lasers, Isometry3qd const& extrinsics, Options options)
        : num_lasers_(num_lasers),
          T_calib_extr_rot_(nie::Isometry3md::FromRotation(extrinsics.rotation().matrix()).ToTransform().cast<float>()),
          options_(options) {}

    [[nodiscard]] std::vector<bool> filter(io::lidar::Returns const& returns, io::lidar::Angles const& angles) const;

private:
    // Apply the rotation of the lidar extrinsics to transform the points from the lidar system to GPS
    [[nodiscard]] pcl::PointCloud<PointT> ApplyCalibration(pcl::PointCloud<PointT> const& points) const;

    uint const num_lasers_;
    Eigen::Transform<float, 3, Eigen::Isometry> const T_calib_extr_rot_;

    Options const options_;
};

}  // namespace nie
