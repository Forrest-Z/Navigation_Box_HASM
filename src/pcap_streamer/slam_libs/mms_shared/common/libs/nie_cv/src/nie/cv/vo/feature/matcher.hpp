/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCHER_HPP
#define NIE_CV_VO_FEATURE_MATCHER_HPP

#include "descriptor.hpp"
#include "keypoint.hpp"
#include "match.hpp"

namespace nie {

/// Abstract class specifying the interface for feature matchers
class Matcher {
public:
    virtual ~Matcher() = default;

    /// Base matching function to find matches between two sets of features.
    /// When the prev_matches are provided, then only those features will be matched. Providing an empty vector will
    /// make the function match all features possible.
    virtual MatchVector Match(
        KeypointVector const& features_a,
        KeypointTypeVector const& feature_types_a,
        DescriptorVector const& descriptors_a,
        KeypointVector const& features_b,
        KeypointTypeVector const& feature_types_b,
        DescriptorVector const& descriptors_b,
        MatchVector const& prev_matches) = 0;

protected:
    Matcher() = default;
};
using MatcherPtr = std::unique_ptr<Matcher>;

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCHER_HPP
