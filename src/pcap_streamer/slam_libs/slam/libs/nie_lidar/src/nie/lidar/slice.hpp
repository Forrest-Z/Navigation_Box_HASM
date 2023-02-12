/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie {

/// Get iterator to where the accumulated distance first exceeds @param{max_distance}.
/// Distance returned is the distance from accumulating @param{first} to @retval.
template <typename Metric, typename Iterator, typename Scalar = typename Metric::Scalar>
[[nodiscard]] std::pair<Iterator, Scalar> SliceByDistance(
    Iterator const first, Iterator const last, Scalar const& max_distance, Metric metric) {
    //
    Scalar distance{0};
    Iterator it = first + 1;
    for (; it != last; ++it) {
        Scalar delta = metric(*it, *(it - 1));
        distance += delta;
        if (distance > max_distance) {
            // Subtract because the accumulating from first to it does not include it
            distance -= delta;
            break;
        }
    }
    return std::make_pair(it, distance);
}

}  // namespace nie
