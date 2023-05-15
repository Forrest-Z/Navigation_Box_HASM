/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
/** LiDAR Sensor Parameters Format
 *
 * This file defines a format to structure, save and load the parameters for a (set of) LiDAR sensor(s)
 * The struct of the LiDAR sensor contains:
 * - id: This is the name that the lidar has e.g. "Velodyne_Front"
 * - ip: This is the ip adress that the lidar is assigned e.g. "192.168.1.202"
 * - cut_angle_start/cut_angle_stop: In between these azimuth angles, the packets should be ignore as the data might be
 * corrupt due to interference with other LiDARs.
 * - extrinsics: This contains the extrinsic calibration of the LiDAR. It is typically saved in an xyz translation and
 * quaternion rotation.
 *
 * The files are written and read using the opencv file io. This means that files can be exported to different file
 * formats. However, it is prefered to use ".json" files.
 *
 * NOTE: This file has a deviation from the google code guide because this interface is needed for OpenCV serialization.
 */

#pragma once

#include <string>
#include <vector>

#include <nie/core/geometry/isometry3.hpp>

namespace nie {

namespace io {

// Definition of LiDAR Types. enum and string vector should match.
enum class LidarType { kVelodyneVLP16 = 0, kVelodyneHDL32, kOusterOS1_32, kOusterOS1_128, kOusterOS2_128, kKitti };
static std::vector<std::string> const kLidarTypeStrings = {
        "Velodyne_VLP16", "Velodyne_HDL32", "Ouster_OS1_32", "Ouster_OS1_128", "Ouster_OS2_128", "KITTI"};
LidarType LidarTypeStringToEnum(std::string const& lidar_type_string);

struct LidarParameters {
    static LidarParameters Read(std::string const& filename);
    void Write(std::string const& filename) const;

    bool operator==(LidarParameters const&) const;

    LidarType type;
    std::string id;
    std::string ip;
    int port;
    // Cut angle defines the angle at which packets should be ignore due to
    // expected interference of other LiDARs
    float cut_angle_start;
    float cut_angle_stop;

    nie::Isometry3qd extrinsics;
};

void Read(std::string const& filename, std::vector<LidarParameters>*);
void Write(std::string const& filename, std::vector<LidarParameters> const&);

/**
 * Find LidarParameters with matching identifier in vector
 *
 * Searches the input vector for a match with the given identifier and returns the result.
 * Will terminate using LOG(FATAL) if no match is found
 *
 * @param lidar_parameters The vector of parameters to be searched
 * @param identifier The identifier string to be matched
 * @result the LidarParameters struct that contains the identifier
 */
LidarParameters FindOrFatalLidarParametersByIdentifier(
        std::vector<LidarParameters> const& lidar_parameters, std::string const& identifier);

/**
 * Read lidar_parameters.json file and returns the LidarParameters struct with matching identifier
 *
 * First reads the input file and stores it in vector.
 * Then searches the vector for a match using FindOrFatalLidarParametersByIdentifer(*).
 * Returns the match if found, otherwise will terminate using LOG(FATAL)
 *
 * @param filename Filename of the input .json file
 * @param identifier The identifier string to be matched
 * @result the LidarParameters struct that contains the identifier
 */
LidarParameters ReadLidarParametersByIdentifier(std::string const& filename, std::string const& identifier);

}  // namespace io

}  // namespace nie
