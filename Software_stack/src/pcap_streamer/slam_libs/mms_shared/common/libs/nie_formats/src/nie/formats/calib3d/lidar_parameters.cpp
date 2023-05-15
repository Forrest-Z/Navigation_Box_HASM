/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_parameters.hpp"

#include <algorithm>

#include <opencv2/core.hpp>

#include "helper_calibration_io.hpp"
namespace cv {

void write(cv::FileStorage& fs, std::string const&, nie::io::LidarParameters const& p) {
    fs << "{";
    fs << "type"
       << nie::io::kLidarTypeStrings[static_cast<std::size_t>(p.type)];  // nie::io::LidarTypeEnumToString(p.type);
    fs << "id" << p.id;
    fs << "ip" << p.ip;
    fs << "port" << p.port;
    fs << "cut_angle" << p.cut_angle_start;
    fs << "cut_angle_stop" << p.cut_angle_stop;
    fs << "extrinsics" << p.extrinsics;
    fs << "}";
}

void read(
        cv::FileNode const& node,
        nie::io::LidarParameters& x,
        nie::io::LidarParameters const& default_value = nie::io::LidarParameters()) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        std::string tmp_type;
        node["type"] >> tmp_type;
        x.type = nie::io::LidarTypeStringToEnum(tmp_type);
        node["id"] >> x.id;
        node["ip"] >> x.ip;
        node["port"] >> x.port;
        node["cut_angle_start"] >> x.cut_angle_start;
        node["cut_angle_stop"] >> x.cut_angle_stop;
        node["extrinsics"] >> x.extrinsics;
    }
}

}  // namespace cv
namespace nie {

namespace io {

LidarType LidarTypeStringToEnum(std::string const& lidar_type_string) {
    auto it = std::find(kLidarTypeStrings.begin(), kLidarTypeStrings.end(), lidar_type_string);
    CHECK(it != kLidarTypeStrings.cend()) << "Unknown LiDAR type: " << lidar_type_string;
    return static_cast<LidarType>(std::distance(kLidarTypeStrings.begin(), it));
}

bool LidarParameters::operator==(LidarParameters const& other) const {
    return std::tie(id, ip, port, type, cut_angle_start, cut_angle_stop, extrinsics) == std::tie(
                                                                                                other.id,
                                                                                                other.ip,
                                                                                                other.port,
                                                                                                other.type,
                                                                                                other.cut_angle_start,
                                                                                                other.cut_angle_stop,
                                                                                                other.extrinsics);
}

LidarParameters LidarParameters::Read(std::string const& filename) {
    std::vector<LidarParameters> x;
    nie::io::Read(filename, &x);
    return x[0];
}

void Read(std::string const& filename, std::vector<LidarParameters>* ret) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    CHECK(storage.isOpened()) << "Unable to read file: " + filename;

    storage["lidars"] >> *ret;

    CHECK(!ret->empty());
    for (auto const& p : *ret) {
        CHECK(p.ip != "") << filename << ": LiDAR should be assigned an ip";
        CHECK(p.port > 1024 && p.port <= 65535) << filename << ": LiDAR should be assigned a valid network port";
    }
}

void LidarParameters::Write(std::string const& filename) const {
    std::vector<LidarParameters> x{};
    x.push_back(*this);
    nie::io::Write(filename, x);
}

void Write(std::string const& filename, std::vector<LidarParameters> const& params) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    CHECK(storage.isOpened()) << "Unable to write file: " + filename;

    storage << "lidars" << params;
}

LidarParameters FindOrFatalLidarParametersByIdentifier(
        std::vector<LidarParameters> const& lidar_parameters, std::string const& identifier) {
    auto it = std::find_if(lidar_parameters.begin(), lidar_parameters.end(), [identifier](const LidarParameters& l) {
        return l.id == identifier;
    });
    if (it == lidar_parameters.end()) {
        LOG(FATAL) << "Could not find Lidar Parameters";
    }
    return *it;
}

LidarParameters ReadLidarParametersByIdentifier(std::string const& filename, std::string const& identifier) {
    std::vector<LidarParameters> lidar_parameters;
    nie::io::Read(filename, &lidar_parameters);
    return FindOrFatalLidarParametersByIdentifier(lidar_parameters, identifier);
}

}  // namespace io

}  // namespace nie