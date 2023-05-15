/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <nie/cv/vo/feature/matcher_lv.hpp>

namespace {

constexpr int kGridSize = 10;

nie::MatcherPtr GenerateMatcher() {
    nie::MatcherLv::Parameters params;
    params.search_distance = 2;
    nie::MatcherPtr matcher = std::make_unique<nie::MatcherLv>(kGridSize, kGridSize, params);
    return matcher;
}

/*
 * . . . . . 5 . . . .
 * . . . . . . . . . .
 * . . . . . . . . . .
 * . 2 . . . . . . . 8
 * . . . . . . . . . .
 * . . . . . . . . . .
 * . . . . . . 9 . . .
 * . . . . . . . . . .
 * . . . . . . . . . .
 * . . . . 7 . . . . .
 */
void GetReferenceSituation(nie::KeypointVector& features, nie::DescriptorVector& descriptors) {
    features.emplace_back(5, 0);
    features.emplace_back(1, 3);
    features.emplace_back(9, 3);
    features.emplace_back(6, 6);
    features.emplace_back(4, 9);

    descriptors.emplace_back(5);
    descriptors.emplace_back(2);
    descriptors.emplace_back(8);
    descriptors.emplace_back(9);
    descriptors.emplace_back(7);
}

struct TestDescription {
    std::string name;
    nie::KeypointVector features;
    nie::KeypointTypeVector feature_types;
    nie::DescriptorVector descriptors;
    nie::MatchVector prev_matches;
    nie::MatchVector matches;
};

/*
 * All test cases are defined relative to the reference situation
 */
std::vector<TestDescription> GenerateTestCases() {
    std::vector<TestDescription> tests;
    {
        TestDescription test{};
        test.name = "No features to match";
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . x . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "No match (based on position)";
        test.features.emplace_back(1, 7);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(1);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . x . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "1 match based on position";
        test.features.emplace_back(1, 5);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(3);
        test.matches.emplace_back(/* previous */ 1, /* current */ 0);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . x . . . . . .
         * . . . . . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "1 match based on descriptor";
        test.features.emplace_back(3, 1);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(3);
        test.matches.emplace_back(/* previous */ 1, /* current */ 0);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . x . . . . . .
         * . . . x . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "2 matches based on same descriptor";
        test.features.emplace_back(3, 1);
        test.features.emplace_back(3, 2);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(3);
        test.descriptors.emplace_back(3);
        test.matches.emplace_back(/* previous */ 1, /* current */ 1);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . x . . . . . .
         * . . . x . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "2 matches based on different descriptor";
        test.features.emplace_back(3, 1);
        test.features.emplace_back(3, 2);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(3);
        test.descriptors.emplace_back(4);
        test.matches.emplace_back(/* previous */ 1, /* current */ 0);
        test.matches.emplace_back(/* previous */ 0, /* current */ 1);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . x . . . . . .
         * . . . x . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "2 matches based on different descriptor, one removed by prev_matches";
        test.features.emplace_back(3, 1);
        test.features.emplace_back(3, 2);
        test.feature_types.resize(test.features.size());
        test.descriptors.emplace_back(3);
        test.descriptors.emplace_back(4);
        test.prev_matches.emplace_back(/* previous */ 7, /* current */ 1);
        test.matches.emplace_back(/* previous */ 1, /* current */ 0);
        tests.push_back(test);
    }
    {
        /*
         * . . . . . 5 . . . .
         * . . . . . . . . . .
         * . . . x . . . . . .
         * . 2 . . . . . . . 8
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . . . 9 . . .
         * . . . . . . . . . .
         * . . . . . . . . . .
         * . . . . 7 . . . . .
         */
        TestDescription test{};
        test.name = "No matches based on feature type";
        test.features.emplace_back(3, 2);
        test.feature_types.emplace_back(nie::KeypointType::kBlobMin);
        test.descriptors.emplace_back(3);
        tests.push_back(test);
    }
    return tests;
}

}  // namespace

TEST(VoMatcherLvTest, Tests) {
    std::vector<TestDescription> tests = GenerateTestCases();
    for (TestDescription& test : tests) {
        nie::KeypointVector features;
        nie::DescriptorVector descriptors;
        GetReferenceSituation(features, descriptors);
        nie::KeypointTypeVector feature_types(features.size());

        nie::MatcherPtr matcher = GenerateMatcher();
        nie::MatchVector actual = matcher->Match(
            features,
            feature_types,
            descriptors,
            test.features,
            test.feature_types,
            test.descriptors,
            test.prev_matches);

        // Compare matches
        EXPECT_EQ(actual.size(), test.matches.size()) << test.name << ": Size mismatch of match vector.";

        std::sort(actual.begin(), actual.end());
        std::sort(test.matches.begin(), test.matches.end());

        for (std::size_t index = 0; index < actual.size(); ++index) {
            EXPECT_EQ(actual[index].index_a, test.matches[index].index_a)
                << test.name << ": Reference feature does not match.";
            EXPECT_EQ(actual[index].index_b, test.matches[index].index_b)
                << test.name << ": Test feature does not match.";
        }
    }
}
