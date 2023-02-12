/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/lidar/euclidean_distance.hpp>
#include <nie/lidar/slice.hpp>

class SliceF : public ::testing::Test {
protected:
    // clang-format off
    std::vector<nie::Isometry3qd> const kPoses{
        // slice 0
        {{0, 0, 0}, {0,0,0,0}},
        {{0.1, 0, 0}, {0,0,0,0}},
        {{0.2, 0, 0}, {0,0,0,0}},
        {{0.3, 0, 0}, {0,0,0,0}},
        {{0.5, 0, 0}, {0,0,0,0}},
        {{1.5, 0, 0}, {0,0,0,0}},
        {{2.0, 0, 0}, {0,0,0,0}},
        {{2.0, 0, 0}, {0,0,0,0}},
        {{2.0, 0, 0}, {0,0,0,0}},
        // slice 1
        {{2.01, 0, 0}, {0,0,0,0}},
        {{3.5, 0, 0}, {0,0,0,0}},
        {{3.99999, 0, 0}, {0,0,0,0}},
        {{4.0, 0, 0}, {0,0,0,0}},
        // slice 2
        {{6.0, 0, 0}, {0,0,0,0}},
        {{8.0, 0, 0}, {0,0,0,0}},
        // slice 3
        {{10.0, 0, 0}, {0,0,0,0}},
        // slice 4
        {{15.0, 0, 0}, {0,0,0,0}},
        {{16.0, 0, 0}, {0,0,0,0}},
        {{17.0, 0, 0}, {0,0,0,0}},

    };
    std::vector<std::vector<nie::Isometry3qd>> const kExpectedSlices{
        {
            {{0, 0, 0}, {0,0,0,0}},
            {{0.1, 0, 0}, {0,0,0,0}},
            {{0.2, 0, 0}, {0,0,0,0}},
            {{0.3, 0, 0}, {0,0,0,0}},
            {{0.5, 0, 0}, {0,0,0,0}},
            {{1.5, 0, 0}, {0,0,0,0}},
            {{2.0, 0, 0}, {0,0,0,0}},
            {{2.0, 0, 0}, {0,0,0,0}},
            {{2.0, 0, 0}, {0,0,0,0}},
        },
        {
            {{2.01, 0, 0}, {0,0,0,0}},
            {{3.5, 0, 0}, {0,0,0,0}},
            {{3.99999, 0, 0}, {0,0,0,0}},
            {{4.0, 0, 0}, {0,0,0,0}},
        },
        {
            {{6.0, 0, 0}, {0,0,0,0}},
            {{8.0, 0, 0}, {0,0,0,0}},
        },
        {
            {{10.0, 0, 0}, {0,0,0,0}}
        },
        {
            {{15.0, 0, 0}, {0,0,0,0}},
            {{16.0, 0, 0}, {0,0,0,0}},
            {{17.0, 0, 0}, {0,0,0,0}},
        }
    };
    // clang-format on
};

TEST_F(SliceF, SlicePoses) {
    double const max_distance = 2;
    auto it = kPoses.cbegin();
    int slice_count = 0;
    while (it != kPoses.cend()) {
        auto result = nie::SliceByDistance(it, kPoses.cend(), max_distance, nie::EuclideanDistance{});
        auto it_next = result.first;
        double distance = result.second;

        EXPECT_LE(distance, max_distance);

        DVLOG(9) << "slice " << slice_count;
        ASSERT_EQ(kExpectedSlices[slice_count].size(), std::distance(it, it_next));

        auto it_expect = kExpectedSlices[slice_count].cbegin();
        auto it_sliced = it;
        for (; it_sliced != it_next; ++it_expect, ++it_sliced) {
            DVLOG(9) << "it_sliced: " << *it_sliced << " == " << *it_expect;
            ASSERT_EQ(*it_sliced, *it_expect);
        }
        ++slice_count;
        it = it_next;
    }
}
