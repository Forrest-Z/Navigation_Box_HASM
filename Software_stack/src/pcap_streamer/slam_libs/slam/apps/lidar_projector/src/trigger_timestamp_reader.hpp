/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef TRIGGER_TIMESTAMP_READER_HPP
#define TRIGGER_TIMESTAMP_READER_HPP

#include <unordered_map>

#include <nie/core/time.hpp>

// Forward declaration
namespace boost {
namespace filesystem {
class path;
}
}  // namespace boost

/**
 * @brief Read the timestamps of all camera triggers from an '.sta' file
 *
 * The file format is described on page 161 of the Inertial Explorer User Manual v8.70
 * See: https://www.novatel.com/assets/Documents/Waypoint/Downloads/Inertial-Explorer-User-Manual-870.pdf
 */
std::unordered_map<unsigned, nie::Timestamp_us> ReadTriggerTimestamps(boost::filesystem::path const& path);

#endif  // TRIGGER_TIMESTAMP_READER_HPP
