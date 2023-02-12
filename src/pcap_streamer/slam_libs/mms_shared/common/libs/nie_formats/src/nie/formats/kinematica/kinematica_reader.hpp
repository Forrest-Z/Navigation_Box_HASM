/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <vector>

namespace nie {

namespace io {

struct KinematicaCsvRecord {
    long unix_time;
    long microseconds;
    double latitude;
    double longitude;
    double height;
    double latitude_sd;
    double longitude_sd;
    double height_sd;
    double velocity_north;
    double velocity_east;
    double roll;
    double pitch;
    double heading;
    double roll_sd;
    double pitch_sd;
    double heading_sd;
};

using KinematicaRecordCollection = std::vector<KinematicaCsvRecord>;

/// Read a csv that was produced by the Kinematica service and return a KinematicaCsvRecord with all fields parsed to
/// their correct types
/// \param kinematica_csv_path path to a csv file created by Kinematica
/// \return KinematicaRecordCollection with all fields parsed to their correct types
KinematicaRecordCollection ReadKinematicaCsv(std::string const& kinematica_csv_path);

}  // namespace io

}  // namespace nie