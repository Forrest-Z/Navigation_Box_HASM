/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <iostream>
#include <nie/lidar/time_intervals/time_intervals_reader.hpp>
#include "time_intervals_common.hpp"

TEST_F(TimeIntervalsTest, ReadIntervals) {
    auto const obtained_intervals =
            nie::io::ReadIntervalsList("/data/aiim/unit_tests_data/slam/time_intervals_test/time_intervals.txt");
    // clang-format off
    std::vector<TimeRange> expected_intervals = ToTimestampPairs(
    {
             {"2018-12-12 02:32:40.000000000", "2018-12-12 02:39:14.010000000"},
             {"2018-12-12 02:39:21.130000000", "2018-12-12 02:39:21.130000000"},
             {"2018-12-12 02:39:21.150000000", "2018-12-12 02:39:21.650000000"},
             {"2018-12-12 02:39:40.175000000", "2018-12-12 02:39:41.120000000"},
             {"2018-12-12 02:44:19.885000000", "2018-12-12 02:45:38.900000000"},
             {"2018-12-12 02:48:49.729999999", "2018-12-12 02:48:57.445000000"},
             {"2018-12-12 02:49:14.969999999", "2018-12-12 02:50:01.854999999"},
             {"2018-12-12 02:51:59.164999999", "2018-12-12 02:52:55.575000000"},
             {"2018-12-12 02:54:38.050000000", "2018-12-12 02:56:03.175000000"},
             {"2018-12-12 02:56:51.940000000", "2018-12-12 02:57:18.735000000"},
             {"2018-12-12 02:59:22.685000000", "2018-12-12 02:59:36.710000000"},
             {"2018-12-12 03:01:08.380000000", "2018-12-12 03:02:42.610000000"},
             {"2018-12-12 03:04:39.625000000", "2018-12-12 03:06:14.090000000"},
             {"2018-12-12 03:08:34.050000000", "2018-12-12 03:09:21.205000000"},
             {"2018-12-12 03:09:27.224999999", "2018-12-12 03:10:04.775000000"},
             {"2018-12-12 03:11:49.315000000", "2018-12-12 03:11:50.000000000"},
             {"2018-12-12 03:12:22.974999999", "2018-12-12 03:13:09.935000000"},
             {"2018-12-12 03:14:31.664999999", "2018-12-12 03:16:47.250000000"},
             {"2018-12-12 03:17:46.159999999", "2018-12-12 03:18:22.419999999"},
             {"2018-12-12 03:20:19.789999999", "2018-12-12 03:21:40.164999999"},
             {"2018-12-12 03:24:03.659999999", "2018-12-12 03:24:42.610000000"},
    });
    // clang-format on
    ASSERT_EQ(obtained_intervals.size(), expected_intervals.size());
    for (size_t i = 0; i < obtained_intervals.size(); ++i) {
        EXPECT_EQ(obtained_intervals[i], expected_intervals[i]) << "Obtained range[" << i << "] is not expected!";
    }
}

TEST_F(TimeIntervalsTest, CheckForFileValidity) {
    // Should not fatal out -----------------
    // Empty interval can be defined if there are weeks number and header.
    ASSERT_NO_FATAL_FAILURE(nie::io::ReadIntervalsList(
            "/data/aiim/unit_tests_data/slam/time_intervals_test/time_intervals-header-only.txt"));
    // --------------------------------------

    // Expected Fatal cases -----------------
    // Check for week and header validity
    ASSERT_DEATH_IF_SUPPORTED(
            nie::io::ReadIntervalsList(
                    "/data/aiim/unit_tests_data/slam/time_intervals_test/time-intervals-invalid-week-number.txt"),
            "");
    ASSERT_DEATH_IF_SUPPORTED(
            nie::io::ReadIntervalsList(
                    "/data/aiim/unit_tests_data/slam/time_intervals_test/time-intervals-invalid-header.txt"),
            "");
    // Empty file is invalid.
    ASSERT_DEATH_IF_SUPPORTED(
            nie::io::ReadIntervalsList("/data/aiim/unit_tests_data/slam/time_intervals_test/time_intervals-empty.txt"),
            "");
    // "Corrupted" or with not supported content file is invalid.
    ASSERT_DEATH_IF_SUPPORTED(
            nie::io::ReadIntervalsList("/data/aiim/unit_tests_data/slam/time_intervals_test/time_intervals-crlf.txt"),
            "");
}