/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "match_filter_local_flow.hpp"

#include <glog/logging.h>

namespace nie {

const MatchFilterLocalFlow::MatchId MatchFilterLocalFlow::kNoMatchId = -1;

void MatchFilterLocalFlow::Filter(
        KeypointVector const& prev_features,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool>* p_filter) const {
    // Sanity checks
    if (parameters_.minimum_number_of_neighbors == 0) {
        return;
    }
    assert(p_filter != nullptr);
    std::vector<bool>& filter = *p_filter;

    // This vector contains the mapping from the vertex id's as known by the
    // triangulation object to the original match id for which the location was
    // added to the triangulation. The vertex id is used as index to address the
    // corresponding match id.
    // Note that in theory there can be multiple features at the same location,
    // but these will get the same triangulation vertex id and will therefore
    // filtered out. The vector could look like:
    //    4: 0
    //    5: 2
    //    6: 1
    //     ...
    std::vector<MatchId> vertex_match_translation;

    // Do the Delaunay triangulation for all features
    Subdiv2D triang = InitTriangulation(matches, features, filter, &vertex_match_translation);

    // Get the edge count of every matched feature
    std::vector<int> edge_counts =
            GetSupportedEdgeCounts(triang, vertex_match_translation, matches, prev_features, features);

    // Set the filter for the matches (true = to be removed)
    for (std::size_t index = 0; index < matches.size(); ++index) {
        if (not filter[index] and edge_counts[index] < parameters_.minimum_number_of_neighbors) {
            filter[index] = true;
        }
    }
}

Subdiv2D MatchFilterLocalFlow::InitTriangulation(
        MatchVector const& matches,
        KeypointVector const& features,
        std::vector<bool> const& filter,
        std::vector<MatchId>* p_vertex_match_translation) const {
    assert(p_vertex_match_translation != nullptr);

    // Create the triangulation object given the image boundaries
    Subdiv2D triangulation(cv::Rect(0, 0, image_width_, image_height_));

    // Add the feature locations of all matches to the triangulation object and
    // store the relation between the acquired index in that triangulation and the
    // original vector index for later reference
    // Note that each addition of a point will update the triangulation
    p_vertex_match_translation->resize(matches.size() + 4, kNoMatchId);
    for (std::size_t index = 0; index < matches.size(); ++index) {
        if (filter[index]) {
            // Will already be filtered, skip
            continue;
        }

        // Get the current feature of this match
        cv::Point2f const& location = features[matches[index].index_b];

        // Add the match location to the delaunay triangulation.
        // Note that the same location will get the same vertex.
        VertexId triangulation_vertex_id = triangulation.insert(location);

        // Use the returned vertex id as known by the triangulation to address
        // the original match id in a vector, to be able to look up later
        (*p_vertex_match_translation)[triangulation_vertex_id] = index;
    }

    return triangulation;
}

std::vector<int> MatchFilterLocalFlow::GetSupportedEdgeCounts(
        Subdiv2D const& triangulation,
        std::vector<MatchId> const& vertex_match_translation,
        MatchVector const& matches,
        KeypointVector const& prev_features,
        KeypointVector const& features) const {
    std::vector<int> edge_counts(matches.size(), 0);

    std::vector<std::pair<VertexId, VertexId>> edges;
    triangulation.getConnectionList(&edges);

    // Loop over all edges of the triangulation
    for (std::pair<VertexId, VertexId> const& vertex_ids : edges) {
        // Look up the match id's that correspond to the edge vertices, but only
        // when it are points we have added
        if (vertex_ids.first >= static_cast<VertexId>(vertex_match_translation.size()) or
            vertex_ids.second >= static_cast<VertexId>(vertex_match_translation.size())) {
            continue;
        }
        MatchId const& match_a_id = vertex_match_translation[vertex_ids.first];
        MatchId const& match_b_id = vertex_match_translation[vertex_ids.second];
        if (match_a_id == kNoMatchId or match_b_id == kNoMatchId) {
            continue;
        }

        // Get the matches
        FeatureMatch const& match_a = matches[match_a_id];
        FeatureMatch const& match_b = matches[match_b_id];

        // For the two edge vertices / two matches, calculate the difference in
        // flow (flow from the previous to the current feature location) as
        // follows:
        //   flow_diff = sum( abs( (a_current - a_prev) - (b_current - b_prev) ) )
        //             = sum( abs(  a_current - a_prev  -  b_current + b_prev  ) )
        Keypoint flow = features[match_a.index_b];
        flow -= prev_features[match_a.index_a];
        flow -= features[match_b.index_b];
        flow += prev_features[match_b.index_a];
        float flow_diff = std::abs(flow.x) + std::abs(flow.y);

        // Increase count of the matches when their difference in flow is under
        // the threshold
        if (flow_diff < parameters_.flow_tolerance) {
            edge_counts[match_a_id]++;
            edge_counts[match_b_id]++;
        }
    }

    return edge_counts;
}

}  // namespace nie
