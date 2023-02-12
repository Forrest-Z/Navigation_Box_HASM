/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "time_intervals_common.hpp"

TEST_F(TimeIntervalsTest, StandStillFilter) {
    // clang-format off
    std::vector<std::pair<std::string, std::string>> to_discard{
            {"2018-01-20 02:58:50.740921600","2018-01-20 02:58:50.940655800"}, // will contain 3 sweeps
            {"2018-01-20 02:58:51.007236744","2018-01-20 02:58:51.008"},       // small intersection to the left, won't discard
            {"2018-01-20 02:58:51.140401820", "2018-01-20 02:58:51.20"},       // large enough intersection to the left, do discard
            {"2018-01-20 02:58:51.273575968", "2018-01-20 02:58:51.28"},       // small intersection to the right, won't discard
            {"2018-01-20 02:58:51.473363170", "2018-01-20 02:58:51.53"},       // large enough intersection to the right, do discard
            {"2018-01-20 02:58:51.606584688", "2018-01-20 02:58:51.673204688"},// exact match, to discard
    };
    // clang-format on
    std::vector<TimeRange> const intervals = ToTimestampPairs(to_discard);
    nie::TimeIntervals stationary_ranges(intervals, 0.8);

    // clang-format off
    std::vector<TimeRange> expected_discards = ToTimestampPairs({
            {"2018-01-20 02:58:50.740923904", "2018-01-20 02:58:50.807501240"},
            {"2018-01-20 02:58:50.807502392", "2018-01-20 02:58:50.874070512"},
            {"2018-01-20 02:58:50.874082032", "2018-01-20 02:58:50.940655800"},
            {"2018-01-20 02:58:51.140401824", "2018-01-20 02:58:51.206990528"},
            {"2018-01-20 02:58:51.473364312", "2018-01-20 02:58:51.539969336"},
            {"2018-01-20 02:58:51.606584688", "2018-01-20 02:58:51.673204688"},
    });
    // clang-format on

    std::vector<TimeRange> obtained_discards;
    std::vector<TimeRange> input_ranges = ToTimestampPairs(kInputRanges_);
    for (auto const& sweep_time_range : input_ranges) {
        if (stationary_ranges.IsMostlyContained(sweep_time_range)) {
            // Check against expected
            obtained_discards.push_back(sweep_time_range);
        }
    }

    ASSERT_EQ(obtained_discards.size(), expected_discards.size()) << "Number of expected discards doesn't match!";
    for (size_t i = 0; i < obtained_discards.size(); ++i) {
        auto const& obtained = obtained_discards[i];
        EXPECT_EQ(obtained, expected_discards[i]) << "Obtained discard element[" << i << "] is not present in the list";
    }
}