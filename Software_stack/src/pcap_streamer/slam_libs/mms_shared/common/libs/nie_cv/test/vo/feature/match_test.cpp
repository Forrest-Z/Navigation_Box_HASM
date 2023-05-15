/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gtest/gtest.h>
#include <numeric>

#include <nie/cv/vo/feature/match.hpp>

TEST(MatchFilterTest, FilterMatchVectorTest) {
    nie::MatchVector matches;
    std::vector<bool> filter;
    matches.emplace_back(0, 0);
    filter.push_back(false);
    matches.emplace_back(1, 0);
    filter.push_back(false);
    matches.emplace_back(2, 0);
    filter.push_back(true);
    matches.emplace_back(3, 0);
    filter.push_back(true);
    matches.emplace_back(4, 0);
    filter.push_back(false);
    matches.emplace_back(5, 0);
    filter.push_back(true);
    matches.emplace_back(6, 0);
    filter.push_back(false);
    matches.emplace_back(7, 0);
    filter.push_back(false);
    matches.emplace_back(8, 0);
    filter.push_back(true);
    nie::MatchVector const orig_matches = matches;

    nie::FilterMatchVector(filter, &matches);

    ASSERT_EQ(matches.size(), filter.size() - std::accumulate(filter.begin(), filter.end(), 0));

    nie::MatchVector::const_iterator result_iter = matches.begin();
    for (std::size_t index = 0; index < orig_matches.size(); ++index) {
        if (not filter[index]) {
            ASSERT_EQ(result_iter->index_a, orig_matches[index].index_a);
            ++result_iter;
        }
    }
}

TEST(MatchVectorIntersectionTest, CombineMatchVectorsTestSingle) {
    nie::MatchVector matches_ab;
    matches_ab.emplace_back(5, 0);
    matches_ab.emplace_back(0, 1);
    matches_ab.emplace_back(8, 6);
    matches_ab.emplace_back(6, 2);
    matches_ab.emplace_back(4, 4);
    matches_ab.emplace_back(3, 3);

    std::sort(matches_ab.begin(), matches_ab.end());
    std::vector<std::vector<std::size_t>> const result = nie::ChainMatches({matches_ab});

    ASSERT_EQ(result.size(), matches_ab.size());

    std::sort(matches_ab.begin(), matches_ab.end());
    for (std::size_t i = 0; i < matches_ab.size(); ++i) {
        ASSERT_EQ(result[i][0], matches_ab[i].index_a);
        ASSERT_EQ(result[i][1], matches_ab[i].index_b);
    }
}

TEST(MatchVectorIntersectionTest, CombineMatchVectorsTestDouble) {
    nie::MatchVector matches_ab, matches_bc;
    std::vector<std::vector<std::size_t>> matches_abc;

    // clang-format off
    matches_ab.emplace_back(5, 0);
    matches_ab.emplace_back(0, 1); matches_bc.emplace_back(1, 9); matches_abc.push_back({0, 1, 9});
                                   matches_bc.emplace_back(8, 4);
    matches_ab.emplace_back(8, 6); matches_bc.emplace_back(6, 8); matches_abc.push_back({8, 6, 8});
    matches_ab.emplace_back(6, 2);
    matches_ab.emplace_back(4, 4);
    matches_ab.emplace_back(3, 3); matches_bc.emplace_back(3, 2); matches_abc.push_back({3, 3, 2});
    // clang-format on

    std::sort(matches_ab.begin(), matches_ab.end());
    std::sort(matches_bc.begin(), matches_bc.end());
    std::vector<std::vector<std::size_t>> const result = nie::ChainMatches({matches_ab, matches_bc});

    ASSERT_EQ(result.size(), matches_abc.size());

    std::sort(matches_abc.begin(), matches_abc.end());
    for (std::size_t i = 0; i < matches_abc.size(); ++i) {
        ASSERT_EQ(result[i][0], matches_abc[i][0]);
        ASSERT_EQ(result[i][1], matches_abc[i][1]);
        ASSERT_EQ(result[i][2], matches_abc[i][2]);
    }
}
