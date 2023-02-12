/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/weiya/image_time_in_day.hpp>


TEST(ImageTimeInDay, GetTimePointFromFilename) {

    // Test value based on their readme (see include)
    double const expected_seconds_in_day = 20345.307907;

    // Parse the GPS time from the given filename
    auto const gps_time = nie::io::weiya::GpsTimeFromFilename("20180519-053847307907-0000000001_L.jpg");

    // Convert to alternative representation of GPS date / time in date
    auto const gps_daytime = nie::ToGPSDayTime(gps_time);

    // Represent time in date as double [in seconds]
    double const seconds_in_day = nie::RepresentDurationAsDouble(gps_daytime.time_in_day);

    // Compare with an epsilon one order of magnitude less than the expected constant
    EXPECT_NEAR(expected_seconds_in_day, seconds_in_day, 1e-7);
}
