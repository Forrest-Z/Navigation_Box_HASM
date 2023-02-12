/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCH_FILTER_HPP
#define NIE_CV_VO_FEATURE_MATCH_FILTER_HPP

#include <memory>

#include "keypoint.hpp"
#include "match.hpp"

namespace nie {

/// Abstract class specifying the interface for feature match filters
class MatchFilter {
public:
    virtual ~MatchFilter() = default;

    /// Given the features that are matched, outlier matches are identified by this function and set true in the filter
    /// vector. Note that the order of the matches is not preserved.
    virtual void Filter(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool>* p_filter) const = 0;

protected:
    MatchFilter() = default;
};
using MatchFilterPtr = std::unique_ptr<MatchFilter>;

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCH_FILTER_HPP
