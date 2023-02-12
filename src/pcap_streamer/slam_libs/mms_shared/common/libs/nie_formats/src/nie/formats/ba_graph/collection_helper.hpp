/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <unordered_map>
#include <vector>

#include "info_ref_collection.hpp"
#include "object_collection.hpp"
#include "pose_collection.hpp"

namespace nie {

namespace io {

// Helper function that returns the maximum PoseRecord id.
PoseId GetMaxPoseId(PoseCollection const& coll);

// Helper function that returns a map (as a vector) to look up the index of a FixedPoseRecord to get the index of the
// corresponding PoseRecord in the PoseCollection.
std::vector<std::size_t> CreateFixedPoseMap(PoseCollection const& coll);

/// Create map from RecordType::id to RecordType wrapped in reference_wrapper.
/// The bracket operator[] cannot be used on the resulting map because
/// std::reference_wrapper is not default constructable. Use map.at() in stead.
/// However, it is possible to assign a reference_wrapped object to a reference without calling .get():
///     RecordType& record = result.at(0);
template <typename Iterator, typename RecordType = std::remove_reference_t<decltype(*std::declval<Iterator>())>>
std::unordered_map<PoseId, std::reference_wrapper<RecordType>> CreateRecordMap(Iterator begin, Iterator end) {
    std::unordered_map<PoseId, std::reference_wrapper<RecordType>> result{};
    for (auto it = begin; it != end; ++it) {
        result.emplace(it->id, std::reference_wrapper<RecordType>{*it});
    }
    return result;
}

/// Get Edges Indices filtered by category.
std::vector<std::size_t> GetEdgesIndicesByCategory(
        std::vector<nie::io::PoseEdgeRecord> const& edges, nie::io::PoseEdgeRecord::Category category);

double GetMedianDistance(PoseCollection const& collection);

}  // namespace io

}  // namespace nie
