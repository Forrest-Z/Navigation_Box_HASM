/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCHER_LV_HPP
#define NIE_CV_VO_FEATURE_MATCHER_LV_HPP

#include "matcher.hpp"

namespace nie {

/*
 * LibViso2-based features matcher
 * Reference: "SteroScan: Dense 3d Reconstruction in Real-time" by Geiger,
 *            Ziegler and Stiller, 2011
 */
class MatcherLv final : public Matcher {
public:
    struct Parameters {
        Parameters()
            : search_distance(50)  // 50, 200
        {}

        int search_distance;
    };

    explicit MatcherLv(int image_width, int image_height, Parameters parameters = Parameters())
        : parameters_(parameters), image_width_(image_width), image_height_(image_height) {}
    ~MatcherLv() override = default;

    MatchVector Match(
        KeypointVector const& features_previous,
        KeypointTypeVector const& feature_types_previous,
        DescriptorVector const& descriptors_previous,
        KeypointVector const& features_current,
        KeypointTypeVector const& feature_types_current,
        DescriptorVector const& descriptors_current,
        MatchVector const& prev_matches) override;

private:
    /*
     * Return the id (position in vector) of the feature in the vector that is
     * closest to the given feature, based on the sum of absolute differences
     * between the descriptors of the given feature and the candidate one. When
     * no match is found, then -1 is returned.
     *
     * The filter indicates the features and descriptors that should be checked
     * for a possible match using their positional indices. The mask keeps track
     * of the features (in the previous image) that are already matched.
     */
    int FindMatch(
        Keypoint const& feature,
        KeypointType const& feature_type,
        FeatureDescriptor const& descriptor,
        KeypointVector const& features,
        KeypointTypeVector const& feature_types,
        DescriptorVector const& descriptors,
        std::vector<std::size_t> const& filter,
        std::vector<bool> const& mask = std::vector<bool>()) const;

    Parameters const parameters_;

    int const image_width_;
    int const image_height_;
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCHER_LV_HPP
