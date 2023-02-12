/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_extrinsics.hpp"

#include <nie/core/filesystem.hpp>
#include <nie/core/string.hpp>

namespace nie {

Isometry3qd ReadLidarExtrinsics(std::string const& filepath) {
    std::fstream file = OpenFile(filepath, std::ios::in);

    std::string line{};

    // First line matches: MDL
    CHECK(std::getline(file, line)) << "Unable to read first lidar extrinsics line.";
    line = nie::Strip(line, "\r\n");
    CHECK(line == "MDL") << "First lidar extrinsics line must be \"MDL\" but got \"" << line << "\" in stead.";

    // Second line has 3 double values
    CHECK(std::getline(file, line)) << "Unable to read second lidar extrinsics line.";
    line = nie::Strip(line, "\r\n");
    std::vector<double> const angles = Split<double>(line, ' ');
    CHECK(angles.size() == 3) << "Second lidar extrinsics line must be 3 doubles but got " << angles.size()
                              << " in stead.";

    // Third line has 3 double values
    CHECK(std::getline(file, line)) << "Unable to read second lidar extrinsics line.";
    line = nie::Strip(line, "\r\n");
    std::vector<double> const translation = Split<double>(line, ' ');
    CHECK(translation.size() == 3) << "Third lidar extrinsics line must be 3 doubles but got " << translation.size()
                                   << " in stead.";

    // Construct isometry from euler angles and translation
    double const x = translation[0];
    double const y = translation[1];
    double const z = translation[2];

    double const r = angles[0];
    double const p = angles[1];
    double const h = angles[2];
    return CalibrationTransform(x, y, z, r, p, h);
}

}  // namespace nie
