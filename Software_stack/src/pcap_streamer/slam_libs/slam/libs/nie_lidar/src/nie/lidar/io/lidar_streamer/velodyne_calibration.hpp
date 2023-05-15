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

namespace velodyne {

class LaserCalibration {
public:
    // Calibration values are zero by default
    explicit LaserCalibration(
            double azimuth = 0.0,
            double vertical = 0.0,
            double distance = 0.0,
            double vert_offset = 0.0,
            double horz_offset = 0.0);

    [[nodiscard]] inline double azimuth() const { return azimuth_; }
    [[nodiscard]] inline double vertical() const { return vertical_; }
    [[nodiscard]] inline double distance() const { return distance_; }
    [[nodiscard]] inline double vert_offset() const { return vert_offset_; }
    [[nodiscard]] inline double horz_offset() const { return horz_offset_; }

    // Values below are precomputed as optimisation
    [[nodiscard]] inline double sin_vert() const { return sin_vert_; }
    [[nodiscard]] inline double cos_vert() const { return cos_vert_; }
    [[nodiscard]] inline double sin_vert_offset() const { return sin_vert_offset_; }
    [[nodiscard]] inline double cos_vert_offset() const { return cos_vert_offset_; }

private:
    double azimuth_;
    double vertical_;
    double distance_;
    double vert_offset_;
    double horz_offset_;

    double sin_vert_;
    double cos_vert_;
    double sin_vert_offset_;
    double cos_vert_offset_;
};

class LidarCalibration {
public:
    explicit LidarCalibration(nie::io::LidarType lidar_type, std::vector<LaserCalibration>&& laser_calibrations);

    [[nodiscard]] inline LidarType lidar_type() const { return lidar_type_; }
    [[nodiscard]] inline uint num_lasers() const { return laser_calibrations_.size(); }
    [[nodiscard]] inline std::vector<LaserCalibration> const& laser_calibrations() const { return laser_calibrations_; }
    [[nodiscard]] inline std::chrono::nanoseconds time_between_block_firings() const {
        return time_between_block_firings_;
    }
    [[nodiscard]] inline std::chrono::nanoseconds time_between_laser_firings() const {
        return time_between_laser_firings_;
    }
    [[nodiscard]] inline std::chrono::nanoseconds time_first_to_last_block() const { return time_first_to_last_block_; }
    [[nodiscard]] inline uint firing_sequences_per_block() const { return firing_sequences_per_block_; }

private:
    LidarType lidar_type_;

    // Calibrations of the individual lasers obtained from the calibration files.
    std::vector<LaserCalibration> laser_calibrations_;

    // LiDAR specific parameters. These should stay constant between all LiDARs of the same type.
    std::chrono::nanoseconds time_between_block_firings_;
    std::chrono::nanoseconds time_between_laser_firings_;
    // Time [in ns] between start of first block and start of last block
    std::chrono::nanoseconds time_first_to_last_block_;
    // Number of firing sequences per data block
    // Currently is not used in the code, but not sure if it should be applied?
    uint firing_sequences_per_block_;
};

[[nodiscard]] LidarCalibration LoadCalibrationFromFile(
        nie::io::LidarType lidar_type, boost::filesystem::path const& calibration_path);
}  // namespace velodyne

}  // namespace io

}  // namespace nie