/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <mutex>
#include <thread>

#include <gtest/gtest.h>

#include <nie/core/scoped_timer.hpp>


TEST(ScopedTimer, SanityCheck) {
    nie::ScopedTimer::SetEnabled(true);
    {
        nie::ScopedTimer timer("sanity_test");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto stats = nie::ScopedTimer::GetStatistics();
    EXPECT_EQ(stats.size(), 1);                 // one thread
    EXPECT_EQ(stats[0].size(), 1);              // single node: root
    EXPECT_EQ(stats[0][0].children.size(), 0);  // no children
    auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(stats[0][0].sum - std::chrono::milliseconds(100)).count();
    EXPECT_LE(diff, 10);
    EXPECT_EQ(stats[0][0].hits, 1);
    EXPECT_EQ(stats[0][0].unique_id, "sanity_test");
}

TEST(ScopedTimer, DISABLED_NotEnabled) {
    nie::ScopedTimer::SetEnabled(false);
    {
        nie::ScopedTimer timer("disabled");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto stats = nie::ScopedTimer::GetStatistics();
    EXPECT_EQ(stats.size(), 1);     // one thread
    EXPECT_EQ(stats[0].size(), 0);  // no nodes
}

// Disabled because it fails unpredictably
TEST(ScopedTimer, DISABLED_ChildSumCheck) {
    int time_epsilon_milli = 1;
    nie::ScopedTimer::SetEnabled(true);
    {
        nie::ScopedTimer timer("root");
        {
            nie::ScopedTimer timer("a");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        {
            nie::ScopedTimer timer("b");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    auto stats = nie::ScopedTimer::GetStatistics();

    EXPECT_EQ(stats[0].size(), 3);              // root, a and b
    EXPECT_EQ(stats[0][0].children.size(), 2);  // 2 children
    auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(stats[0][0].sum - std::chrono::milliseconds(5)).count();
    EXPECT_LE(diff, time_epsilon_milli);
    EXPECT_EQ(stats[0][0].hits, 1);
    EXPECT_EQ(stats[0][0].unique_id, "root");

    // 1st child
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(
               stats[0][stats[0][0].children[0]].sum - std::chrono::milliseconds(2))
               .count();
    EXPECT_LE(diff, time_epsilon_milli);
    EXPECT_EQ(stats[0][0].hits, 1);
    EXPECT_EQ(stats[0][stats[0][0].children[0]].unique_id, "a");

    // 2nd child
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(
               stats[0][stats[0][0].children[1]].sum - std::chrono::milliseconds(1))
               .count();
    EXPECT_LE(diff, time_epsilon_milli);
    EXPECT_EQ(stats[0][0].hits, 1);
    EXPECT_EQ(stats[0][stats[0][0].children[1]].unique_id, "b");
}

// Disabled because it fails unpredictably
TEST(ScopedTimer, DISABLED_LoopIncrement) {
    int time_epsilon_milli = 1;
    nie::ScopedTimer::SetEnabled(true);
    {
        nie::ScopedTimer timer("root");
        {
            nie::ScopedTimer timer("a");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        for (int i = 0; i < 5; ++i) {
            nie::ScopedTimer timer("b");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    auto stats = nie::ScopedTimer::GetStatistics();

    EXPECT_EQ(stats[0].size(), 3);              // root, a and b
    EXPECT_EQ(stats[0][0].children.size(), 2);  // 2 children
    auto diff = std::chrono::duration<double, std::milli>(stats[0][0].sum - std::chrono::milliseconds(27));
    EXPECT_GE(diff.count(), 0);
    EXPECT_LE(diff.count(), time_epsilon_milli);

    // 2nd child was called 5x
    diff = std::chrono::duration<double, std::milli>(
            stats[0][stats[0][0].children[1]].sum - std::chrono::milliseconds(25));
    EXPECT_GE(diff.count(), 0);
    EXPECT_LE(diff.count(), time_epsilon_milli);

    EXPECT_EQ(stats[0][stats[0][0].children[1]].hits, 5);
    EXPECT_EQ(stats[0][stats[0][0].children[1]].unique_id, "b");
}
