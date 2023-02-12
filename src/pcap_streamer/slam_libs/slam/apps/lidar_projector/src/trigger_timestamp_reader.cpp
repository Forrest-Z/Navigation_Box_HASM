/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "trigger_timestamp_reader.hpp"

#include <nie/core/filesystem.hpp>
#include <nie/core/string.hpp>

std::unordered_map<unsigned, nie::Timestamp_us> ReadTriggerTimestamps(boost::filesystem::path const& path) {
    std::fstream file = nie::OpenFile(path, std::ios::in);

    // Read the single line header and verify that has the expected format
    std::string header;
    std::getline(file, header);

    if (!nie::StartsWith(header, "$STAINFO")) {
        std::stringstream ss;
        ss << "File " << path << " does not have the expected format";

        throw std::runtime_error(ss.str());
    }

    // Define empty event id / timestamp
    unsigned event_id = 0;
    nie::Timestamp_us event_timestamp;

    // Loop over all events in the file
    std::string line;
    std::unordered_map<unsigned, nie::Timestamp_us> events;
    while (std::getline(file, line)) {
        // Strip whitespace
        nie::Trim(&line);

        // Interpret line
        if (line.back() == '{') {
            // Reset values
            event_id = 0;
            event_timestamp = nie::Timestamp_us();
        } else if (line.back() == '}') {
            // Store the event
            events[event_id] = event_timestamp;
        } else {
            // Find split position for key value pair
            std::size_t split_pos = line.find(':');
            if (split_pos == std::string::npos) {
                throw std::runtime_error("Unable to parse key value pair");
            }

            // Extract key and value and trim whitespace
            std::string const& key = nie::Trim(line.substr(0, split_pos));
            std::string const& value = nie::Trim(line.substr(split_pos + 1, std::string::npos));

            if (key == "Event") {
                // Extract event id from value
                event_id = std::stoull(nie::Strip(value, '"')) - 1;
            } else if (key == "GTim") {
                // Split value into value list
                auto const values = nie::Split<std::string>(value, ' ');

                // Extract GPS week from values
                auto const gps_weeks = std::chrono::weeks{std::stoul(values[1])};

                // Extract GPS seconds in week, handle integer part [in s] and fractional part [in μs] separately
                auto const gps_seconds_in_week =
                    nie::ParseFractionalDuration<std::chrono::seconds, std::chrono::microseconds>(values[0]);

                nie::GPSWeekTime<std::chrono::microseconds> gps_week_time{gps_weeks, gps_seconds_in_week};

                // Convert to a GPS timestamp
                event_timestamp = nie::ToGPSTime(gps_week_time);
            } else if (key == "Desc") {
                // Extract description from value
                std::string const description = nie::Strip(value, '"');

                if (description != "CAMERA PULSE") {
                    throw std::runtime_error("Encountered unexpected event description '" + description + "'");
                }
            } else {
                throw std::runtime_error("Encountered unexpected key '" + key + "'");
            }
        }
    }

    return events;
}
