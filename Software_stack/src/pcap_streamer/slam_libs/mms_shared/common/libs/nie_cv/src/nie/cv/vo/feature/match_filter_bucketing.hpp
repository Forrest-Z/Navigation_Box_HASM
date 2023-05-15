/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCH_FILTER_BUCKETING_HPP
#define NIE_CV_VO_FEATURE_MATCH_FILTER_BUCKETING_HPP

#include "match_filter.hpp"

namespace nie {

class MatchFilterBucketing final : public MatchFilter {
public:
    enum class ReductionStrategy { kFirst, kRandom };

    struct Parameters {
        Parameters() : grid_cell_size(15.), maximum_number_of_matches(2), strategy(ReductionStrategy::kRandom) {}

        float grid_cell_size;
        int maximum_number_of_matches;
        ReductionStrategy strategy;
    };

    explicit MatchFilterBucketing(int image_width, int image_height, Parameters parameters = Parameters())
        : parameters_(parameters), image_width_(image_width), image_height_(image_height) {}
    ~MatchFilterBucketing() override = default;

    void Filter(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool>* p_filter) const override;

private:
    Parameters const parameters_;

    int const image_width_;
    int const image_height_;
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCH_FILTER_BUCKETING_HPP
