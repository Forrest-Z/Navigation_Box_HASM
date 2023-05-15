/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "matcher_lv.hpp"

#include <nie/core/geometry/conversion.hpp>

#include "nie/cv/vo/visual_odometry.hpp"

namespace nie {

MatchVector MatcherLv::Match(
        KeypointVector const& features_previous,
        KeypointTypeVector const& feature_types_previous,
        DescriptorVector const& descriptors_previous,
        KeypointVector const& features_current,
        KeypointTypeVector const& feature_types_current,
        DescriptorVector const& descriptors_current,
        MatchVector const& prev_matches) {
    assert(features_previous.size() == feature_types_previous.size());
    assert(features_previous.size() == descriptors_previous.size());
    assert(features_current.size() == feature_types_current.size());
    assert(features_current.size() == descriptors_current.size());

    if (features_previous.empty() or features_current.empty()) {
        return {};
    }

    MatchVector matches;
    matches.reserve(features_previous.size());

    // Keep track which features are currently already matched and which are not yet (for the round trip match)
    std::vector<bool> to_match_in_current(features_current.size(), true);
    // The features that we are actually trying to find a match
    std::vector<bool> to_match_in_previous(features_previous.size(), true);

    if (not prev_matches.empty()) {
        std::fill(std::begin(to_match_in_previous), std::end(to_match_in_previous), false);
        for (FeatureMatch const& m : prev_matches) {
            to_match_in_previous[m.index_b] = true;
        }
    }

    // As a preparation step, the features will be added to a spatial grid
    // to speed up the look up of features close to a certain point.
    // TODO: Now the grids are recreated every time, but the grid of previous features could be kept and reused.
    // TODO: The grid of the current features are also used by the match filter MatchFilterBucketing.
    SpatialGrid<std::size_t, 2, float> const grid_previous =
            CreateSpatialGridVo(parameters_.search_distance, features_previous, to_match_in_previous);
    SpatialGrid<std::size_t, 2, float> const grid_current =
            CreateSpatialGridVo(parameters_.search_distance, features_current, to_match_in_current);

    // First step is to match all features from the previous frame with the
    // ones from the current frame. This is done by matching in a circle.
    // If the current feature x is matched to previous feature y, and
    // previous feature y is on its turn also matched to current feature x,
    // then the match x-y is accepted.
    for (int id_previous = 0; id_previous < static_cast<int>(features_previous.size()); ++id_previous) {
        if (not to_match_in_previous[id_previous]) continue;

        std::vector<std::size_t> indices;
        grid_current.GetBox(ConvertPoint(features_previous[id_previous]), parameters_.search_distance, &indices);

        // Find match of current feature in the previous features
        int id_current = FindMatch(
                features_previous[id_previous],
                feature_types_previous[id_previous],
                descriptors_previous[id_previous],
                features_current,
                feature_types_current,
                descriptors_current,
                indices,
                to_match_in_current);

        if (id_current == -1) {
            continue;
        }

        grid_previous.GetBox(ConvertPoint(features_current[id_current]), parameters_.search_distance, &indices);

        // Find match of found previous feature in the current features
        int new_id_previous = FindMatch(
                features_current[id_current],
                feature_types_current[id_current],
                descriptors_current[id_current],
                features_previous,
                feature_types_previous,
                descriptors_previous,
                indices,
                to_match_in_previous);

        // If the "round trip" matching succeeded, then add match
        if (new_id_previous != -1 and new_id_previous == id_previous) {
            matches.emplace_back(id_previous, id_current);
            to_match_in_current[id_current] = false;
            to_match_in_previous[id_previous] = false;
        }
    }

    return matches;
}

int MatcherLv::FindMatch(
        Keypoint const& feature,
        KeypointType const& feature_type,
        FeatureDescriptor const& descriptor,
        KeypointVector const& features,
        KeypointTypeVector const& feature_types,
        DescriptorVector const& descriptors,
        std::vector<std::size_t> const& filter,
        std::vector<bool> const& mask) const {
    // Candidate feature id/result that has least error compared compared to
    // given feature
    int result = -1;
    int32_t min_error = std::numeric_limits<int32_t>::max();

    // Loop over eligible candidates
    bool const mask_empty = mask.empty();
    for (std::size_t id : filter) {
        Keypoint const& candidate_point = features[id];
        // Before considering the encountered feature, it should
        //  - be of same type, and
        //  - not be matched already, and
        //  - within search window
        if (feature_type == feature_types[id] and (mask_empty or mask[id]) and
            (std::abs(feature.x - candidate_point.x) <= float(parameters_.search_distance) and
             std::abs(feature.y - candidate_point.y) <= float(parameters_.search_distance))) {
            // Calculate the sum of absolute differences of the two descriptors
            int32_t error = 0;
            auto const& candidate_descriptor = descriptors[id];
            for (int index = 0; index < kFeatureDescriptorSize; ++index) {
                error += std::abs(descriptor[index] - candidate_descriptor[index]);
            }

            // The encountered feature is the new candidate match feature, when
            //  - it is the first point being encountered, or
            //  - the new descriptor matches better, or
            //  - the new descriptor matches just as good, but the new feature is closer
            if (error < min_error or
                (error == min_error and cv::norm(feature - candidate_point) < cv::norm(feature - features[result]))) {
                min_error = error;
                result = id;
            }
        }
    }

    return result;
}

}  // namespace nie
