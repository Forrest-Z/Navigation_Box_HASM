/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "visual_odometry_configuration.hpp"

#include <nie/core/scoped_timer.hpp>

#include <cassert>

namespace nie {

void VisualOdometryConfiguration::DetectFeatures(
    cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) const {
    ScopedTimer timer("VisualOdometryConfiguration::DetectFeatures");

    for (DetectorPtr const& feature_detector : feature_detectors) {
        if (feature_detector) {
            feature_detector->Detect(image, p_features, p_feature_types);
        }
    }
}

DescriptorVector VisualOdometryConfiguration::DescribeFeatures(
    cv::Mat const& image, KeypointVector const& features) const {
    ScopedTimer timer("VisualOdometryConfiguration::DescribeFeatures");
    if (feature_descriptor) {
        return feature_descriptor->Describe(image, features);
    } else {
        return {};
    }
}

MatchVector VisualOdometryConfiguration::MatchFeatures(
    KeypointVector const& prev_features,
    KeypointTypeVector const& prev_feature_types,
    DescriptorVector const& prev_descriptors,
    KeypointVector const& features,
    KeypointTypeVector const& feature_types,
    DescriptorVector const& descriptors,
    MatchVector const& prev_matches) const {
    ScopedTimer timer("VisualOdometryConfiguration::MatchFeatures");
    if (feature_matcher) {
        return feature_matcher->Match(
            prev_features, prev_feature_types, prev_descriptors, features, feature_types, descriptors, prev_matches);
    } else {
        return {};
    }
}

void VisualOdometryConfiguration::FilterMatches(
    KeypointVector const& feature_a, KeypointVector const& feature_b, MatchVector* p_matches) const {
    ScopedTimer timer("VisualOdometryConfiguration::FilterMatches");

    assert(p_matches != nullptr);

    if (p_matches->empty()) {
        // Nothing to filter
        return;
    }

    // Remove outlier matches using different strategies
    // detail: using a filter is faster than indices based operations (in combination with CopyIf, RemoveIf)
    std::vector<bool> filter(p_matches->size(), false);  // true = to be removed
    for (MatchFilterPtr const& match_filter : match_filters) {
        if (match_filter) {
            match_filter->Filter(feature_a, feature_b, *p_matches, &filter);
        }
    }
    FilterMatchVector(filter, p_matches);

    // Called one time when comparing two frames, but size (40k-80k) can easily be reduced by 85%
    if (not p_matches->empty()) {
        p_matches->shrink_to_fit();
    }
}

}  // namespace nie
