/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/core/time.hpp>

#include "nie/lidar/range_utils.hpp"

namespace nie {
///
/// Struct to keep track of ordered time intervals to check for intersection against incoming intervals.
/// It is meant to check if any interval on a series of timestamp ranges intersects one of the intervals stored in this
/// structure.
/// To avoid linear searches, it keeps the current relevant interval to be checked in a queue fashion, moving to the
/// next one once the incoming intervals are greater than it.
///
struct TimeIntervals {
    typedef std::pair<nie::Timestamp_ns, nie::Timestamp_ns> TimeRange;

    explicit TimeIntervals(std::vector<TimeRange> const& intervals, double const intersection_ratio = 0.8)
        : intervals_(intervals), intersection_ratio_threshold_(intersection_ratio) {}

    /// \brief Checks if the given TimeRange is contained to a high ratio by the current relevant interval.
    /// If it is greater than the current interval, the check is repeated against the next ones in the stored TimeRange
    /// vector. This implementation assumes both stored and incoming ranges to be in ascending order.
    /// \param range The incoming new range to check against the current time interval.
    /// \return bool stating if the given range is contained or not (considering the intersection_ratio_threshold_).
    bool IsMostlyContained(TimeRange const& range) {
        if (current_index_ >= intervals_.size()) {
            // There are no intervals to check against
            return false;
        }

        // Check if the given range is ahead of the current interval.
        // This is assuming that every new call to this function will provide a crescent range (assumed ordered).
        // TODO (TB) : Support unordered incoming ranges?
        while (intervals_[current_index_].second <= range.first) {
            ++current_index_;
            if (current_index_ >= intervals_.size()) {
                // If the given range is beyond the last interval, there will be no more intersections.
                return false;
            }
        }

        double const ratio = nie::range_utils::IntersectionRatio(
                {intervals_[current_index_].first.time_since_epoch().count(),
                 intervals_[current_index_].second.time_since_epoch().count()},
                {range.first.time_since_epoch().count(), range.second.time_since_epoch().count()});
        return ratio >= intersection_ratio_threshold_;
    }
    void Reset() { current_index_ = 0; }

private:
    size_t current_index_ = 0;
    std::vector<TimeRange> const& intervals_;
    double const intersection_ratio_threshold_;
};

}  // namespace nie
