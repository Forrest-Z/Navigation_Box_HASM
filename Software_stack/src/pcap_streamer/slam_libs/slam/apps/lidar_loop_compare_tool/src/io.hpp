/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <array>

#include <nie/core/hash.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#pragma once

struct PoseSet {
    nie::io::PoseCollection unfiltered;
    nie::io::PoseCollection filtered;
    nie::io::PoseCollection closure;
};

std::vector<PoseSet> ReadPoseFiles(
        std::string const& unfiltered_paths, std::string const& filtered_paths, std::string const& closure_paths);

struct ClosureGroundTruth {
    // If the loop closure succeeded, then the success flag is set to true and the improved is not of interest. When the
    // loop closure did not succeed, then success is false, but the improved indicates whether the result improved the
    // alignment or not.
    bool success;
    bool improved;
};
using ClosureGroundTruthMap =
        std::unordered_map<std::pair<nie::io::PoseId, nie::io::PoseId>, ClosureGroundTruth, nie::PairHash>;

ClosureGroundTruthMap ReadGroundTruth(std::string const& path);

struct CompareCount {
    std::size_t same;
    std::size_t diff;
};
struct GroundTruthCompareResult {
    std::size_t ref_count;
    std::vector<CompareCount> test_counts;
};
struct Statistics {
    // Original candidate matches identified
    std::vector<std::size_t> original_matches;

    // Candidate matches left after filtering
    std::vector<std::size_t> filtered_matches;

    // Matches for which the loop_closer succeeded
    std::vector<std::size_t> succeeded_matches;

    // Succeeded matches which are confirmed to be good
    GroundTruthCompareResult gt_confirmed_matches;

    // Succeeded matches which are confirmed to be improved, but not good
    GroundTruthCompareResult gt_improved_matches;

    // Succeeded matches which are confirmed to have failed
    GroundTruthCompareResult gt_failed_matches;

    // This data structure will be plot by this function as follows:
    //
    //   Statistics
    //     Counts and percentile differences/fractions in brackets
    //                                     Ref                      1                      2
    // ----------------------------------------------------------------------------------------
    //            original matches           9           9  (+  0.0%)           7  (- 22.2%)
    //            filtered matches           4           4  (+  0.0%)           4  (+  0.0%)
    //           succeeded matches           3           3  (+  0.0%)           3  (+  0.0%)
    // ----------------------------------------------------------------------------------------
    //    confirmed matches - same           1           0   (  0.0%)           1   (100.0%)
    //                      - diff                       1   (100.0%)           0   (  0.0%)
    //     improved matches - same           1           1   (100.0%)           1   (100.0%)
    //                      - diff                       0   (  0.0%)           0   (  0.0%)
    //       failed matches - same           1           0   (  0.0%)           0   (  0.0%)
    //                      - diff                       1   (100.0%)           1   (100.0%)
    // ----------------------------------------------------------------------------------------
    void Print(bool print_ground_truth, std::ostream& out = std::cout) const;
};
