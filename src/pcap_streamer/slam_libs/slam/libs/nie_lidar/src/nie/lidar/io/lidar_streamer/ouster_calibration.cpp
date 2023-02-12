/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "ouster_calibration.hpp"

#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <nie/core/geometry/rotation.hpp>

namespace nie {

namespace io {

namespace ouster {

// The Ouster devices have the same intrinsics file layout.
LidarCalibration LoadCalibrationFromFile(boost::filesystem::path const& calibration_path) {
    boost::property_tree::ptree root;
    boost::property_tree::read_json(calibration_path.string(), root);

    LidarCalibration calibration;
    calibration.lidar_origin_to_beam_origin_mm = root.get<double>("lidar_origin_to_beam_origin_mm");

    // Populate the vectors. Store the values in radians as doubles
    auto const read_vector_from_json_field = [&root](auto const& field, auto& vec) {
        for (auto const& element : root.get_child(field)) {
            vec.push_back(nie::Deg2Rad(std::stod(element.second.data())));
        }
    };

    read_vector_from_json_field("beam_altitude_angles", calibration.beam_altitude_radians);
    read_vector_from_json_field("beam_azimuth_angles", calibration.beam_azimuth_radians);

    return calibration;
}

}  // namespace ouster

}  // namespace io

}  // namespace nie
