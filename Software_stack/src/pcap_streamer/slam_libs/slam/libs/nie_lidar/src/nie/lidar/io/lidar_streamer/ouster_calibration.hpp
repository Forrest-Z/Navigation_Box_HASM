/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <chrono>

#include <glog/logging.h>

#include <nie/core/filesystem.hpp>
#include <nie/formats/calib3d/lidar_parameters.hpp>  // LidarType

namespace nie {

namespace io {

namespace ouster {

struct LaserCalibration {
    double beam_altitude_radians;
    double beam_azimuth_radians;
};

struct LidarCalibration {
    double lidar_origin_to_beam_origin_mm;
    std::vector<double> beam_altitude_radians;
    std::vector<double> beam_azimuth_radians;
};

[[nodiscard]] LidarCalibration LoadCalibrationFromFile(boost::filesystem::path const& calibration_path);

}  // namespace ouster

}  // namespace io

}  // namespace nie
