/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_to_csv.hpp"

#include "tool/io.hpp"

#include <nie/core/filesystem.hpp>
#include <nie/core/time.hpp>

// TODO(jbr): Customer specific "temporary" code.
void ConvertToCsv() {
    InPathExistsOrFatal(nie::io::graph::Extension<nie::io::PoseCollection>());
    CheckOutPathsLocationsOrFatal();

    boost::filesystem::path path;
    GetAndCheckOutPathsForExtensionOrFatal(".csv", &path);

    nie::io::PoseCollection collection;
    ReadData(&collection);

    std::string header = "seconds_in_day,northing,easting";

    nie::WriteLines(path, collection.poses, header, [](nie::io::PoseRecord const& p) -> std::string {
        auto const day_time = nie::ToGPSDayTime(p.timestamp);
        return std::to_string(std::chrono::duration<double>(day_time.time_in_day).count()) + "," +
               std::to_string(p.isometry.translation().y()) + "," + std::to_string(p.isometry.translation().x());
    });

    LOG(INFO) << "csv file written.";
}
