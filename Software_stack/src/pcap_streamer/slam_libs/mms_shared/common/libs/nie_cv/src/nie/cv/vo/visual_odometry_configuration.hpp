/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_VISUAL_ODOMETRY_CONFIGURATION_HPP
#define NIE_CV_VO_VISUAL_ODOMETRY_CONFIGURATION_HPP

#include "feature/descriptor_extractor.hpp"
#include "feature/detector.hpp"
#include "feature/match_filter.hpp"
#include "feature/matcher.hpp"

namespace nie {

class VisualOdometryConfiguration {
public:
    VisualOdometryConfiguration(
        std::vector<DetectorPtr> detectors,
        DescriptorExtractorPtr descriptor,
        MatcherPtr matcher,
        std::vector<MatchFilterPtr> filters) noexcept
        : feature_detectors(std::move(detectors)),
          feature_descriptor(std::move(descriptor)),
          feature_matcher(std::move(matcher)),
          match_filters(std::move(filters)) {}

    VisualOdometryConfiguration(VisualOdometryConfiguration&& other) noexcept
        : feature_detectors(std::move(other.feature_detectors)),
          feature_descriptor(std::move(other.feature_descriptor)),
          feature_matcher(std::move(other.feature_matcher)),
          match_filters(std::move(other.match_filters)) {}

    void DetectFeatures(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) const;

    DescriptorVector DescribeFeatures(cv::Mat const& image, KeypointVector const& features) const;

    MatchVector MatchFeatures(
        KeypointVector const& prev_features,
        KeypointTypeVector const& prev_feature_types,
        DescriptorVector const& prev_descriptors,
        KeypointVector const& features,
        KeypointTypeVector const& feature_types,
        DescriptorVector const& descriptors,
        MatchVector const& prev_matches) const;

    // Filter the features matches
    void FilterMatches(KeypointVector const& feature_a, KeypointVector const& feature_b, MatchVector* p_matches) const;

private:
    std::vector<DetectorPtr> feature_detectors;
    DescriptorExtractorPtr feature_descriptor;
    MatcherPtr feature_matcher;
    std::vector<MatchFilterPtr> match_filters;
};

}  // namespace nie

#endif  // NIE_CV_VO_VISUAL_ODOMETRY_CONFIGURATION_HPP
