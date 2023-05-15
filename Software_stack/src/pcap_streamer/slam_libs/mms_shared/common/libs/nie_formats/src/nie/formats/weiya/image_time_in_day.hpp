/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_WEIYA_IMAGE_TIME_IN_DAY_HPP
#define NIE_FORMATS_WEIYA_IMAGE_TIME_IN_DAY_HPP

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/string.hpp>
#include <nie/core/time.hpp>

// clang-format off
// Match equation between images and relevant line in .post file
//
// Take image 20180519-053847307907-0000000001_L.jpg for example:
//
// 1)	The format of the file name is as YYYYMMDD-HHMMSSZZZZZZ-XXXXXXX, whose middle part is indicating the time stamp of the file.
// 2)	Convert HHMMSSZZZZZZ to seconds of day by following
//
//      seconds of day = 3600 * HH + 60 * MM + SS + ZZZZZZ * 10e-6
//
// For example, 053847307907 is converted to 20327.307907
//
// 3)	Adjust seconds of day by adding 18 seconds
//
//        The result of 053847307907 is 20345.307907 seconds
//
// 4)	In the relevant .post file, we could find 20345.307907 is between 20345.300 and 20345.310.
//
// Seqnum	GPSTime	Northing	Easting	H-ELL	Latitude	Longitude	HzSpeed	Rool	Pitch	Heading
// 31410	20345.300	3370068.3796	545341.5812	18.132	30.44977456820	114.47206843569	2.5906	-0.12445	-3.26807	-1.56400
// 31411	20345.310	3770068.4056	545341.5810	18.132	30.44977480267	114.47206843479	2.5968	-0.12326	-3.26467	-1.56330
// clang-format on

namespace nie {
namespace io {
namespace weiya {

/// Obtain the UTC time represented in the filename, and return it as a time on the GPS clock
inline Timestamp_ns GpsTimeFromFilename(boost::filesystem::path const& path) {

    // Obtain filename without path and without extension
    auto const filename = path.stem().string();

    // Example filename:
    //
    //   20190403-040344536071-0000000001_L
    //   YYYYMMDD-HHMMSSZZZZZZ-SEQNUMXXXX_L (L for left, R for right)

    // We are only interested in the date / time, so strip the rest
    std::istringstream iss(filename.substr(0, 21));

    // UTC time in seconds and fractional seconds need to be parsed separately, see:
    //   https://stackoverflow.com/questions/54048426/parsing-subsecond-date-with-howard-hinnant-date-library
    std::chrono::utc_seconds utc_time;
    unsigned us;
    iss >> std::chrono::parse("%Y%m%d-%H%M%2S", utc_time) >> us;

    // Verify that parsing was successful
    CHECK(!iss.fail()) << "Weiya image filename parser could not parse UTC date / time in filename " << path;

    // Combine UTC time with fraction in microseconds and cast to GPS time (Then return implicitly converts to ns)
    return std::chrono::clock_cast<std::chrono::gps_clock>(utc_time + std::chrono::microseconds(us));
}

}  // namespace weiya
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_WEIYA_IMAGE_TIME_IN_DAY_HPP
