/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "match_filter_bucketing.hpp"

#include <nie/cv/vo/visual_odometry.hpp>

namespace nie {

namespace detail {

SpatialGridReductionStrategy TranslateStrategy(MatchFilterBucketing::ReductionStrategy const& strategy) {
    if (strategy == MatchFilterBucketing::ReductionStrategy::kRandom) {
        return SpatialGridReductionStrategy::kRandom;
    }
    return SpatialGridReductionStrategy::kFirst;
}

}  // namespace detail

void MatchFilterBucketing::Filter(
        KeypointVector const&,
        KeypointVector const& features,
        MatchVector const& matches,
        std::vector<bool>* p_filter) const {
    assert(p_filter != nullptr);
    std::vector<bool>& filter = *p_filter;

    // Do the binning, only for the not-to-be-removed matches
    // TODO(MvB): mask should be used instead of filter in all match filtering activities
    std::vector<bool> mask = filter;
    mask.flip();
    SpatialGridVo const grid = CreateSpatialGridVo(parameters_.grid_cell_size, features, matches, mask);

    // Lower the number of matches per grid cell to the maximum, returns the
    // selected matches
    std::vector<std::size_t> selected_ids;
    grid.Reduce(parameters_.maximum_number_of_matches, detail::TranslateStrategy(parameters_.strategy), &selected_ids);

    // Set the filter for the matches (true = to be removed)
    // Has to be done via temporary variable as the selected ids are known (not the removed ones)
    std::vector<bool> tmp_filter(matches.size(), true);
    for (std::size_t selected_id : selected_ids) {
        tmp_filter[selected_id] = false;
    }
    std::swap(filter, tmp_filter);
}

}  // namespace nie
