/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <nie/lidar/time_intervals/time_intervals.hpp>

class TimeIntervalsTest : public testing::Test {
public:
    using TimeRange = typename nie::TimeIntervals::TimeRange;

    std::vector<TimeRange> ToTimestampPairs(std::vector<std::pair<std::string, std::string>> const& intervals) {
        std::vector<TimeRange> time_intervals;

        for (auto const& interval : intervals) {
            std::stringstream ss(interval.first);
            nie::Timestamp_ns begin;
            std::chrono::from_stream(ss, "%F %T", begin);
            ss.str(interval.second);
            nie::Timestamp_ns end;
            std::chrono::from_stream(ss, "%F %T", end);
            time_intervals.emplace_back(begin, end);
        }
        return time_intervals;
    }

protected:
    // clang-format off
    std::vector<std::pair<std::string, std::string>> const kInputRanges_{
            {"2018-01-20 02:58:50.674331552", "2018-01-20 02:58:50.740921600"},
            {"2018-01-20 02:58:50.740923904", "2018-01-20 02:58:50.807501240"},
            {"2018-01-20 02:58:50.807502392", "2018-01-20 02:58:50.874070512"},
            {"2018-01-20 02:58:50.874082032", "2018-01-20 02:58:50.940655800"},
            {"2018-01-20 02:58:50.940667320", "2018-01-20 02:58:51.007235592"},
            {"2018-01-20 02:58:51.007236744", "2018-01-20 02:58:51.073809512"},
            {"2018-01-20 02:58:51.073812968", "2018-01-20 02:58:51.140400672"},
            {"2018-01-20 02:58:51.140401824", "2018-01-20 02:58:51.206990528"},
            {"2018-01-20 02:58:51.206991680", "2018-01-20 02:58:51.273575968"},
            {"2018-01-20 02:58:51.273577120", "2018-01-20 02:58:51.340169432"},
            {"2018-01-20 02:58:51.340179800", "2018-01-20 02:58:51.406765240"},
            {"2018-01-20 02:58:51.406766392", "2018-01-20 02:58:51.473363160"},
            {"2018-01-20 02:58:51.473364312", "2018-01-20 02:58:51.539969336"},
            {"2018-01-20 02:58:51.539971640", "2018-01-20 02:58:51.606582384"},
            {"2018-01-20 02:58:51.606584688", "2018-01-20 02:58:51.673204688"},
            {"2018-01-20 02:58:51.673206992", "2018-01-20 02:58:51.739836168"},
            {"2018-01-20 02:58:51.739838472", "2018-01-20 02:58:51.806479056"},
            {"2018-01-20 02:58:51.806480208", "2018-01-20 02:58:51.873112840"},
            {"2018-01-20 02:58:51.873115144", "2018-01-20 02:58:51.939762792"},
            {"2018-01-20 02:58:51.939773160", "2018-01-20 02:58:52.006427680"},
            {"2018-01-20 02:58:52.006428832", "2018-01-20 02:58:52.073072872"},
            {"2018-01-20 02:58:52.073083240", "2018-01-20 02:58:52.139735456"},
            {"2018-01-20 02:58:52.139736608", "2018-01-20 02:58:52.206396928"},
            {"2018-01-20 02:58:52.206398080", "2018-01-20 02:58:52.273053752"},
    };

    // clang-format on
};