/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gtest/gtest.h>

#include <nie/cv/vo/feature/match_filter_local_flow.hpp>

class VoFeatureMatchFilterLocalFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        nie::MatchFilterLocalFlow::Parameters params;
        params.minimum_number_of_neighbors = 2;
        params.flow_tolerance = 0.1;
        filter_ = std::make_unique<nie::MatchFilterLocalFlow>(10, 10, params);
    }

    void DoTest(
        nie::KeypointVector const& features,
        nie::MatchVector const& matches,
        std::vector<bool> const& filter = std::vector<bool>()) {
        SetTestCase(features, matches, filter);

        std::vector<bool> tmp_filter(matches.size(), false);
        filter_->Filter(features_, features_, matches_, &tmp_filter);
        nie::FilterMatchVector(tmp_filter, &matches_);

        // First simple check of sizes
        ASSERT_EQ(matches_.size(), expected_matches_.size())
            << "The number of matches that were filtered is not as expected.";

        // Filtering does not preserve the order of the matches
        std::sort(matches_.begin(), matches_.end());

        // Second detailed check of entries
        for (std::size_t i = 0; i < matches_.size(); ++i) {
            ASSERT_EQ(matches_[i].index_a, expected_matches_[i].index_a);
            ASSERT_EQ(matches_[i].index_b, expected_matches_[i].index_b);
        }
    }

private:
    void SetTestCase(
        nie::KeypointVector const& features, nie::MatchVector const& matches, std::vector<bool> const& filter) {
        features_ = features;
        matches_ = matches;

        // Set expectations
        expected_matches_.clear();

        if (filter.empty()) {
            expected_matches_ = matches;
        } else {
            for (std::size_t i = 0; i < filter.size(); ++i) {
                if (not filter[i]) {
                    expected_matches_.push_back(matches_[i]);
                }
            }
        }

        std::sort(expected_matches_.begin(), expected_matches_.end());
    }

    nie::MatchFilterPtr filter_;

    nie::KeypointVector features_;
    nie::MatchVector matches_, expected_matches_;
};

TEST_F(VoFeatureMatchFilterLocalFlowTest, BasicTest) {
    /*
     * . . . . . . . . . .
     * . . . . 0 . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . 2
     * . 1 . . . . . . . .
     * . . . . . 3 . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . 4 . . . . . .
     *
     * Feature 3 is at the centre of the four others in the triangulation
     */
    nie::KeypointVector features;
    features.emplace_back(4, 1);  // feature 0
    features.emplace_back(1, 4);  // feature 1
    features.emplace_back(9, 3);  // feature 2
    features.emplace_back(5, 5);  // feature 3
    features.emplace_back(3, 9);  // feature 4

    nie::MatchVector matches;
    // All features going to itself will support each other and will not be filtered
    matches.emplace_back(0, 0);
    matches.emplace_back(1, 1);
    matches.emplace_back(2, 2);
    matches.emplace_back(3, 3);
    matches.emplace_back(4, 4);

    DoTest(features, matches);
}

TEST_F(VoFeatureMatchFilterLocalFlowTest, FlowTest) {
    /*
     * . . . . . . . . . .
     * . . . . 0 . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . 2
     * . 1 . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . 3 . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . 4 . . . . . .
     */
    nie::KeypointVector features;
    features.emplace_back(4, 1);  // feature 0
    features.emplace_back(1, 4);  // feature 1
    features.emplace_back(9, 3);  // feature 2
    features.emplace_back(6, 6);  // feature 3
    features.emplace_back(3, 9);  // feature 4

    nie::MatchVector matches;
    // Matches that are supported by enough neighbouring matches will not be filtered
    matches.emplace_back(0, 1);
    matches.emplace_back(2, 3);
    matches.emplace_back(3, 4);

    DoTest(features, matches);
}

TEST_F(VoFeatureMatchFilterLocalFlowTest, NoFlowTest) {
    /*
     * . . . . . . . . . .
     * . . . . 0 . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . 2
     * . 1 . . . . . . . .
     * . . . . . 3 . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . 4 . . . . . .
     *
     * Feature 3 is at the centre of the four others in the triangulation
     */
    nie::KeypointVector features;
    features.emplace_back(4, 1);  // feature 0
    features.emplace_back(1, 4);  // feature 1
    features.emplace_back(9, 3);  // feature 2
    features.emplace_back(5, 5);  // feature 3
    features.emplace_back(3, 9);  // feature 4

    nie::MatchVector matches;
    std::vector<bool> filtered;
    // All features going to the same position will be support by the others and will not be filtered
    matches.emplace_back(3, 4);
    filtered.push_back(true);
    matches.emplace_back(4, 3);
    filtered.push_back(true);
    // Matches that do not have enough neighbouring matches will be filtered
    matches.emplace_back(0, 5);
    filtered.push_back(true);
    matches.emplace_back(1, 2);
    filtered.push_back(true);

    DoTest(features, matches, filtered);
}

TEST_F(VoFeatureMatchFilterLocalFlowTest, DuplicateTest) {
    /*
     * . . . . . . . . . .
     * . . . . 0 . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . 1 . . . . . . . .
     * . . . . . 2 . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     * . . . . . . . . . .
     */
    nie::KeypointVector features;
    features.emplace_back(4, 1);  // feature 0
    features.emplace_back(1, 4);  // feature 1
    features.emplace_back(5, 5);  // feature 2
    features.emplace_back(5, 5);  // feature 3 -> duplicate of 2

    nie::MatchVector matches;
    std::vector<bool> filtered;
    // All features going to itself will support each other and will not be filtered
    matches.emplace_back(0, 0);
    filtered.push_back(false);
    matches.emplace_back(1, 1);
    filtered.push_back(false);
    matches.emplace_back(2, 2);
    filtered.push_back(true);
    matches.emplace_back(3, 3);
    filtered.push_back(false);

    DoTest(features, matches, filtered);
}
