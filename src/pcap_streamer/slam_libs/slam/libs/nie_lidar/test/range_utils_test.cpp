/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <sstream>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <nie/lidar/range_utils.hpp>

typedef std::tuple<std::pair<int, int>, std::pair<int, int>, bool> Interval;
typedef std::tuple<std::pair<int, int>, std::pair<int, int>, double> PartialInterval;

std::string PrintPartialIntervals(PartialInterval const& val) {
    std::stringstream ss;
    ss << "([" << std::get<0>(val).first << " " << std::get<0>(val).second << "], [" << std::get<1>(val).first << " "
       << std::get<1>(val).second << "], " << std::get<2>(val) << ")" << std::endl;
    return ss.str();
}

std::string PrintIntervals(Interval const& val) {
    std::stringstream ss;
    ss << "([" << std::get<0>(val).first << " " << std::get<0>(val).second << "], [" << std::get<1>(val).first << " "
       << std::get<1>(val).second << "], " << std::boolalpha << std::get<2>(val) << ")" << std::endl;
    return ss.str();
}

TEST(RangeUtilsTest, Intersects) {
    std::vector<Interval> expected_results{
            {{-10, 10}, {-5, 5}, true},   // [..{...}...]
            {{-1, 1}, {-5, 5}, true},     // {...[..]...}
            {{-10, -2}, {-5, 5}, true},   // {...[..}...]
            {{2, 10}, {-5, 5}, true},     // [...{..]...}
            {{-5, 5}, {-5, 5}, true},     // [{.........}]
            {{1, 2}, {1, 2}, true},       // [{.}]
            {{1, 1}, {1, 1}, true},       // [{}]
            {{-10, -5}, {-5, 5}, false},  // {...}[...]
            {{5, 10}, {-5, 5}, false},    // [...]{...}
            {{-10, -8}, {-5, 5}, false},  // {...}..[...]
            {{8, 10}, {-5, 5}, false},    // [...]..{...}
    };

    for (size_t i = 0; i < expected_results.size(); ++i) {
        Interval const& interval = expected_results[i];
        EXPECT_EQ(std::get<2>(interval), nie::range_utils::Intersects(std::get<0>(interval), std::get<1>(interval)))
                << " for #" << i << " : " << PrintIntervals(interval);
    }
}

TEST(RangeUtilsTest, Contains) {
    std::vector<Interval> expected_results{
            {{-10, 10}, {-5, 5}, true},   // [..{...}...]
            {{-1, 1}, {-5, 5}, false},    // {...[..]...}
            {{-10, -2}, {-5, 5}, false},  // {...[..}...]
            {{2, 10}, {-5, 5}, false},    // [...{..]...}
            {{-5, 5}, {-5, 5}, true},     // [{.........}]
            {{1, 2}, {1, 2}, true},       // [{.}]
            {{1, 1}, {1, 1}, true},       // [{}]
            {{-10, -5}, {-5, 5}, false},  // {...}[...]
            {{5, 10}, {-5, 5}, false},    // [...]{...}
            {{-10, -8}, {-5, 5}, false},  // {...}..[...]
            {{8, 10}, {-5, 5}, false},    // [...]..{...}
    };

    for (size_t i = 0; i < expected_results.size(); ++i) {
        Interval const& interval = expected_results[i];
        EXPECT_EQ(std::get<2>(interval), nie::range_utils::Contains(std::get<0>(interval), std::get<1>(interval)))
                << " for #" << i << " : " << PrintIntervals(interval);
    }
}

TEST(RangeUtilsTest, IntersectionRatio) {
    std::vector<PartialInterval> expected_results{
            {{-10, 10}, {-5, 5}, 1.0},  // [..{...}...]
            {{-1, 1}, {-5, 5}, 0.6},    // {...[..]...}
            {{-10, -2}, {-5, 5}, 0.3},  // {...[..}...]
            {{2, 10}, {-5, 5}, 0.3},    // [...{..]...}
            {{-5, 5}, {-5, 5}, 1.0},    // [{.........}]
            {{1, 2}, {1, 2}, 1.0},      // [{.}]
            {{1, 1}, {1, 1}, 1.0},      // [{}]
            {{-10, -5}, {-5, 5}, 0.0},  // {...}[...]
            {{5, 10}, {-5, 5}, 0.0},    // [...]{...}
            {{-10, -8}, {-5, 5}, 0.0},  // {...}..[...]
            {{8, 10}, {-5, 5}, 0.0},    // [...]..{...}
    };

    for (size_t i = 0; i < expected_results.size(); ++i) {
        PartialInterval const& interval = expected_results[i];
        EXPECT_EQ(
                std::get<2>(interval),
                nie::range_utils::IntersectionRatio(std::get<0>(interval), std::get<1>(interval)))
                << " for #" << i << " : " << PrintPartialIntervals(interval);
    }
}