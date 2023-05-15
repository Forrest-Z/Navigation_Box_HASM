/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <random>

#include <gtest/gtest.h>
#include <nie/lidar/range_finder.hpp>

class RangeFinderF : public ::testing::Test {
protected:
    std::vector<int> const kValues{0, 1, 2, 4, 8, 16, 32};
    std::vector<int> const kValuesNotSorted{3, 2, 6, 5, 8, 1, 4, 7, 9};
};

TEST_F(RangeFinderF, SortedLess) {
    std::vector<int> vec = kValuesNotSorted;
    std::sort(vec.begin(), vec.end(), std::less{});
    nie::RangeFinder<int> range_finder{vec};
    for (auto const& val : vec) {
        auto [first, second, success] = range_finder(val);
        ASSERT_TRUE(success);
        ASSERT_EQ(*first, val);
        ASSERT_EQ(*second, val);
    }
}
TEST_F(RangeFinderF, SortedGreater) {
    std::vector<int> vec = kValuesNotSorted;
    std::sort(vec.begin(), vec.end(), std::greater{});
    nie::RangeFinder<int, std::greater> range_finder{vec};
    for (auto const& val : vec) {
        auto [first, second, success] = range_finder(val);
        ASSERT_TRUE(success);
        ASSERT_EQ(*first, val);
        ASSERT_EQ(*second, val);
    }
}

TEST_F(RangeFinderF, SameValue) {
    nie::RangeFinder<int> range_finder{kValues};

    for (auto const& val : kValues) {
        auto [first, second, success] = range_finder(val);
        ASSERT_TRUE(success);
        ASSERT_EQ(*first, val);
        ASSERT_EQ(*second, val);
    }
}

TEST_F(RangeFinderF, SameValueReversed) {
    nie::RangeFinder<int> range_finder{kValues};

    for (auto it = kValues.crbegin(); it != kValues.crend(); ++it) {
        auto val = *it;
        auto [first, second, success] = range_finder(val);
        ASSERT_TRUE(success);
        ASSERT_EQ(*first, val);
        ASSERT_EQ(*second, val);
    }
}

TEST_F(RangeFinderF, SameValueRandom) {
    nie::RangeFinder<int> range_finder{kValues};

    std::vector<int> shuffled_values(kValues.cbegin(), kValues.cend());
    auto rng = std::default_random_engine{};
    std::shuffle(shuffled_values.begin(), shuffled_values.end(), rng);

    for (auto const& val : shuffled_values) {
        auto [first, second, success] = range_finder(val);
        ASSERT_TRUE(success);
        ASSERT_EQ(*first, val);
        ASSERT_EQ(*second, val);
    }
}

TEST_F(RangeFinderF, SeekRangeRandom) {
    nie::RangeFinder<int> range_finder{kValues};

    for (int val = 0; val <= *kValues.crbegin(); ++val) {
        auto [first, second, success] = range_finder(val);

        if (kValues.cend() !=
            std::find_if(kValues.cbegin(), kValues.cend(), [&val](auto pose_record) { return pose_record == val; })) {
            ASSERT_TRUE(success);
            ASSERT_EQ(*first, val);
            ASSERT_EQ(*second, val);
        } else {
            ASSERT_TRUE(success);
            ASSERT_LT(*first, val);
            ASSERT_GT(*second, val);
        }
    }
}

TEST_F(RangeFinderF, SeekRangeEdgeCaseBegin) {
    nie::RangeFinder<int> range_finder{kValues};

    int const val = *kValues.cbegin();

    auto [first, second, success] = range_finder(val);

    int const expected_before = *kValues.cbegin();
    int const expected_after = *kValues.cbegin();

    ASSERT_TRUE(success);
    ASSERT_EQ(*first, expected_before);
    ASSERT_EQ(*second, expected_after);
}

TEST_F(RangeFinderF, SeekRangeEdgeCaseEnd) {
    nie::RangeFinder<int> range_finder{kValues};

    int const val = *kValues.crbegin();

    auto [first, second, success] = range_finder(val);

    int const expected_before = *kValues.crbegin();
    int const expected_after = *kValues.crbegin();

    ASSERT_TRUE(success);
    ASSERT_EQ(*first, expected_before);
    ASSERT_EQ(*second, expected_after);
}

TEST_F(RangeFinderF, SeekRangeOutOfBounds) {
    nie::RangeFinder<int> range_finder{kValues};

    {
        auto [first, second, success] = range_finder(kValues.front() - 1);

        ASSERT_FALSE(success);
        ASSERT_EQ(first, kValues.cend());
        ASSERT_EQ(second, kValues.cend());
    }

    {
        auto [first, second, success] = range_finder(kValues.back() + 1);

        ASSERT_FALSE(success);
        ASSERT_EQ(first, kValues.cend());
        ASSERT_EQ(second, kValues.cend());
    }
}

TEST_F(RangeFinderF, SeekRangeInBetween) {
    nie::RangeFinder<int> range_finder{kValues};

    for (size_t i = 0; i < kValues.size() - 1; ++i) {
        int const val = kValues[i] + 1;
        auto [first, second, success] = range_finder(val);

        ASSERT_TRUE(success);
        if (std::find(kValues.cbegin(), kValues.cend(), val) != kValues.cend()) {
            ASSERT_EQ(*first, val);
            ASSERT_EQ(*second, val);
        } else {
            ASSERT_EQ(*first, kValues[i]);
            ASSERT_EQ(*second, kValues[i + 1]);
        }
    }
}
