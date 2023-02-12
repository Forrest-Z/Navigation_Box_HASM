/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "velodyne_calibration.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <nie/core/geometry/rotation.hpp>
#include <nie/core/glog.hpp>

#include "fixed_angle.hpp"
#include "velodyne_packets.hpp"

namespace nie {

namespace io {

namespace velodyne {

LaserCalibration::LaserCalibration(
        double azimuth, double vertical, double distance, double vert_offset, double horz_offset)
    : azimuth_(azimuth),
      vertical_(vertical),
      distance_(distance),
      vert_offset_(vert_offset),
      horz_offset_(horz_offset) {
    auto const vertical_rad = nie::Deg2Rad(vertical_);

    sin_vert_ = std::sin(vertical_rad);
    cos_vert_ = std::cos(vertical_rad);

    sin_vert_offset_ = vert_offset_ * sin_vert_;
    cos_vert_offset_ = vert_offset_ * cos_vert_;
}

LidarCalibration::LidarCalibration(nie::io::LidarType lidar_type, std::vector<LaserCalibration>&& laser_calibrations)
    : lidar_type_(lidar_type), laser_calibrations_(std::move(laser_calibrations)) {
    switch (lidar_type_) {
        case LidarType::kVelodyneVLP16: {
            // https://velodynelidar.com/wp-content/uploads/2019/09/63-9276-Rev-C-VLP-16-Application-Note-Packet-Structure-Timing-Definition.pdf
            time_between_block_firings_ = std::chrono::nanoseconds(55296);
            time_between_laser_firings_ = std::chrono::nanoseconds(2304);
            firing_sequences_per_block_ = 2;
            break;
        }
        case LidarType::kVelodyneHDL32: {
            // https://velodynelidar.com/wp-content/uploads/2019/09/63-9277-Rev-D-HDL-32E-Application-Note-Packet-Structure-Timing-Definition.pdf
            time_between_block_firings_ = std::chrono::nanoseconds(46080);
            time_between_laser_firings_ = std::chrono::nanoseconds(1152);
            firing_sequences_per_block_ = 1;
            break;
        }
        default: {
            LOG(FATAL) << "Lidar Type: " << nie::io::kLidarTypeStrings[size_t(lidar_type_)]
                       << " is not supported (yet)!";
        }
    }
    time_first_to_last_block_ = time_between_block_firings_ * (kBlocksPerPacket - 1);
    CHECK_NE(num_lasers(), 0) << "Found 0 enabled lasers in the Calibration file.";
}

LidarCalibration LoadCalibrationFromFile(
        nie::io::LidarType lidar_type, boost::filesystem::path const& calibration_path) {
    boost::property_tree::ptree pt;
    try {
        read_xml(calibration_path.string(), pt, boost::property_tree::xml_parser::trim_whitespace);
    } catch (boost::exception const&) {
        LOG(FATAL) << "Error reading calibration file " << calibration_path;
    }

    // Find number of enabled lasers
    // NOTE(tvds): We are assuming that the enabled lasers will all be from index 0 to x where x is the number of
    // lasers. This is the case for all the configuration files we have seen so far. If this would not be the case
    // anymore, we will need a more complicated implementation (linking enabled_ with points_)
    std::int32_t num_lasers = 0;
    for (auto const& item : pt.get_child("boost_serialization.DB.enabled_")) {
        if (item.first != "item") continue;

        std::string const& cal_value = item.second.data();
        if (cal_value == "1") {
            num_lasers++;
        }
    }

    std::vector<LaserCalibration> laser_calibrations(static_cast<std::size_t>(num_lasers));

    // Fill calibration
    for (auto const& v : pt.get_child("boost_serialization.DB.points_")) {
        // Skip value if we are not interested in it
        if (v.first != "item") continue;

        auto const& points = v.second;
        for (auto const& px : points) {
            // Skip value if we are not interested in it
            if (px.first != "px") continue;

            auto const& calibration_data = px.second;
            std::int32_t index{};

            double azimuth{};
            double vertical{};
            double distance{};
            double vert_offset{};
            double horz_offset{};

            try {
                for (auto const& item : calibration_data) {
                    std::string const& cal_key = item.first;
                    std::string const& cal_value = item.second.data();

                    if (cal_key == "id_")
                        index = std::stoi(cal_value);
                    else if (cal_key == "rotCorrection_")
                        azimuth = std::stod(cal_value);
                    else if (cal_key == "vertCorrection_")
                        vertical = std::stod(cal_value);
                    else if (cal_key == "distCorrection_")
                        distance = std::stod(cal_value) / 100;  // Convert from [cm] to [m]
                    else if (cal_key == "vertOffsetCorrection_")
                        vert_offset = std::stod(cal_value) / FixedAngle<>::kStepsPerDegree;
                    else if (cal_key == "horizOffsetCorrection_")
                        horz_offset = std::stod(cal_value) / FixedAngle<>::kStepsPerDegree;
                }
            } catch (std::exception& e) {
                nie::LogExceptionAndFatal(e);
            }

            // Stop if we reached the number of lasers
            if (index >= num_lasers) {
                break;
            }

            laser_calibrations[index] = LaserCalibration(azimuth, vertical, distance, vert_offset, horz_offset);
        }
    }

    return LidarCalibration(lidar_type, std::move(laser_calibrations));
}

}  // namespace velodyne

}  // namespace io

}  // namespace nie