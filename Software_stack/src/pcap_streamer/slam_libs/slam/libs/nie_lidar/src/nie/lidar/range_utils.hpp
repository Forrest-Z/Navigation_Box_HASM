/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie {
namespace range_utils {
/// \brief Checks if there is an intersection between two ranges, assuming left-bounded intervals.
///
/// @param lhs left-bounded interval to check for intersection
/// @param rhs left-bounded interval to check for intersection
/// @return bool true if intersection between lhs and rhs is not empty
///
template <typename T>
inline bool Intersects(std::pair<T, T> const& lhs, std::pair<T, T> const& rhs) {
    return (lhs.first < rhs.second && lhs.second > rhs.first) || lhs.first == rhs.first;
}

/// \brief Checks if the lhs interval contains the rhs one, assuming left-bounded intervals.
///
/// @param lhs left-bounded interval to check if contains
/// @param rhs left-bounded interval to check if is contained
/// @return bool true if lhs contains rhs
///
template <typename T>
inline bool Contains(std::pair<T, T> const& lhs, std::pair<T, T> const& rhs) {
    return (lhs.first <= rhs.first && lhs.second >= rhs.second);
}

/// \brief Calculates the "ratio" of intersection between two ranges, assuming left-bounded intervals,
///        meaning how much of the rhs interval is intersected by the lhs interval.
/// @param lhs left-bounded interval to check for intersection
/// @param rhs left-bounded interval to check for intersection
/// @return double The intersection ratio between lhs and rhs, 0.0 if no intersection,
///                and > 0.0 if partially intersected or >1.0 if fully contained.
///
inline double IntersectionRatio(std::pair<double, double> const& lhs, std::pair<double, double> const& rhs) {
    if (!Intersects(lhs, rhs)) {
        return 0.0;
    }
    return std::min(
            1.0,
            std::min(std::abs(rhs.first - lhs.second), std::abs(rhs.second - lhs.first)) /
                    std::abs(rhs.second - rhs.first));
}
}  // namespace range_utils
}  // namespace nie