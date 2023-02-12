/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "ground_truth_compare.hpp"

#include <numeric>

#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/geometry/rotation.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "io.hpp"

namespace {

void AddGroundTruthStatisticsRefCounts(ClosureGroundTruthMap const& ground_truth, Statistics* stats) {
    stats->gt_confirmed_matches.ref_count = std::accumulate(
            ground_truth.cbegin(), ground_truth.cend(), 0, [](std::size_t const sum, auto const& iter) -> std::size_t {
                ClosureGroundTruth const& data = iter.second;
                return sum + data.success;
            });
    stats->gt_improved_matches.ref_count = std::accumulate(
            ground_truth.cbegin(), ground_truth.cend(), 0, [](std::size_t const sum, auto const& iter) -> std::size_t {
                ClosureGroundTruth const& data = iter.second;
                return sum + (!data.success && data.improved);
            });
    stats->gt_failed_matches.ref_count = std::accumulate(
            ground_truth.cbegin(), ground_truth.cend(), 0, [](std::size_t const sum, auto const& iter) -> std::size_t {
                ClosureGroundTruth const& data = iter.second;
                return sum + (!data.success && !data.improved);
            });
}

bool CompareTransformation(
        nie::Isometry3qd const& ref,
        nie::Isometry3qd const& tst,
        double const translation_threshold,
        double const rotation_threshold) {
    nie::Isometry3qd const diff = ref.Delta(tst);
    double dt = diff.translation().norm();
    double dr = nie::Rad2Deg(Eigen::AngleAxisd(diff.rotation()).angle());
    return dt < translation_threshold && dr < rotation_threshold;
}

std::unordered_map<std::pair<nie::io::PoseId, nie::io::PoseId>, std::size_t, nie::PairHash> CreateEdgeRecordMap(
        std::vector<PoseSet> const& pose_sets) {
    std::unordered_map<std::pair<nie::io::PoseId, nie::io::PoseId>, std::size_t, nie::PairHash> result;
    for (std::size_t i = 0; i < pose_sets.front().closure.edges.size(); ++i) {
        auto const& edge = pose_sets.front().closure.edges[i];
        auto const key = std::make_pair(edge.id_begin, edge.id_end);

        bool const inserted = result.insert({key, i}).second;
        CHECK(inserted) << "Not expecting duplicate edge records, based on the id's.";
    }
    return result;
}

void AddGroundTruthStatisticsTestCounts(
        std::vector<PoseSet> const& pose_sets,
        std::unordered_map<std::pair<nie::io::PoseId, nie::io::PoseId>, std::size_t, nie::PairHash> const&
                edge_ids_to_edge_index_map,
        ClosureGroundTruthMap const& ground_truth,
        double const translation_threshold,
        double const rotation_threshold,
        Statistics* stats) {
    auto const& ref_edges = pose_sets.front().closure.edges;

    // Loop over all loop closure pose files
    for (std::size_t i = 1; i < pose_sets.size(); ++i) {
        auto const& tst_edges = pose_sets[i].closure.edges;

        stats->gt_confirmed_matches.test_counts.emplace_back();
        stats->gt_improved_matches.test_counts.emplace_back();
        stats->gt_failed_matches.test_counts.emplace_back();

        // Helper function to update the count whether the edge transformation is the same or different
        auto const increment_count = [](bool const is_same, CompareCount* pair) {
            (is_same ? pair->same : pair->diff)++;
        };

        // Loop over all loop closure edges in this file
        for (nie::io::PoseEdgeRecord const& tst_edge : tst_edges) {
            auto const key = std::make_pair(tst_edge.id_begin, tst_edge.id_end);

            auto const gt_iter = ground_truth.find(key);
            if (gt_iter != ground_truth.cend()) {
                ClosureGroundTruth const& gt = gt_iter->second;
                nie::Isometry3qd const& T_ref = ref_edges[edge_ids_to_edge_index_map.at(key)].isometry;

                bool const is_same =
                        CompareTransformation(T_ref, tst_edge.isometry, translation_threshold, rotation_threshold);
                if (gt.success) {
                    increment_count(is_same, &stats->gt_confirmed_matches.test_counts.back());
                } else {
                    if (gt.improved) {
                        increment_count(is_same, &stats->gt_improved_matches.test_counts.back());
                    } else {
                        increment_count(is_same, &stats->gt_failed_matches.test_counts.back());
                    }
                }
            }
        }
    }
}

}  // anonymous namespace

void AddGroundTruthStatistics(
        std::vector<PoseSet> const& pose_sets,
        ClosureGroundTruthMap const& ground_truth,
        double const translation_threshold,
        double const rotation_threshold,
        Statistics* stats) {
    AddGroundTruthStatisticsRefCounts(ground_truth, stats);

    // Build map from id's of edge to its index in the vector and check uniqueness of edges
    auto const edge_ids_to_edge_index_map = CreateEdgeRecordMap(pose_sets);

    // Additionally check that the ground truth data all refers to given edges
    for (auto const& record : ground_truth) {
        CHECK(edge_ids_to_edge_index_map.find(record.first) != edge_ids_to_edge_index_map.cend())
                << "Ground truth data refers to edge not present in first given closure loops pose file.";
    }

    AddGroundTruthStatisticsTestCounts(
            pose_sets, edge_ids_to_edge_index_map, ground_truth, translation_threshold, rotation_threshold, stats);
}
