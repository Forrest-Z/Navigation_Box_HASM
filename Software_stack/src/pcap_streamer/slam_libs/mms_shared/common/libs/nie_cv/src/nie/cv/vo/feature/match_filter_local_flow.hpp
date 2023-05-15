/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCH_FILTER_LOCAL_FLOW_HPP
#define NIE_CV_VO_FEATURE_MATCH_FILTER_LOCAL_FLOW_HPP

#include "match_filter.hpp"

#include "nie/cv/subdiv_2d.hpp"

namespace nie {

/*
 * LibViso2-based feature match outlier removal
 * Reference: "SteroScan: Dense 3d Reconstruction in Real-time" by Geiger,
 *            Ziegler and Stiller, 2011
 *
 * Strategy is to take all feature matches and form a Delauney triangulation
 * based on the current feature location of every match. This enables
 * identifying the neighboring matches of a given match. A match is accepted
 * (not marked for filtering) when a match has the given number of neighbors
 * that only differ in flow less than the given threshold. Here flow stands for
 * the feature motion between the images.
 *
 * Remarks:
 *   - When two features have the same location, then one will be filtered out.
 */
class MatchFilterLocalFlow final : public MatchFilter {
public:
    struct Parameters {
        Parameters() : minimum_number_of_neighbors(2), flow_tolerance(5) {}

        int minimum_number_of_neighbors;
        float flow_tolerance;
    };

    explicit MatchFilterLocalFlow(int image_width, int image_height, Parameters parameters = Parameters())
        : parameters_(parameters), image_width_(image_width), image_height_(image_height) {}
    ~MatchFilterLocalFlow() override = default;

    void Filter(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool>* p_filter) const override;

private:
    using MatchId = int;
    using VertexId = int;
    static MatchId const kNoMatchId;

    /*
     * Create a Delauney triangulation object of the current feature location of
     * all matches. The trianguation object is returned and the vertex_ids map
     * is filled. This map translates the triangulation vertex id's to the
     * corresponding match id's.
     */
    Subdiv2D InitTriangulation(
        MatchVector const& matches,
        KeypointVector const& features,
        std::vector<bool> const& filter,
        std::vector<MatchId>* p_vertex_match_translation) const;

    /*
     * From the triangulation object, all edges are processed. What is returned
     * is a vector of how many matches are supporting a certain match. For every
     * edge the difference in flow for the two neighboring matches is
     * calculated. When this difference is below the given threshold, then the
     * result vector is incremented for both matched of the edge.
     */
    std::vector<int> GetSupportedEdgeCounts(
        Subdiv2D const& triangulation,
        std::vector<MatchId> const& vertex_match_translation,
        MatchVector const& matches,
        KeypointVector const& prev_features,
        KeypointVector const& features) const;

    Parameters const parameters_;

    int const image_width_;
    int const image_height_;
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCH_FILTER_LOCAL_FLOW_HPP
