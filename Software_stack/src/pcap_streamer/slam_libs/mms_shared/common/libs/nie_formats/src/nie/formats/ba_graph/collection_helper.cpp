/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "collection_helper.hpp"

namespace nie {

namespace io {

namespace detail {

template <typename Record>
std::size_t FindIndex(std::vector<Record> const& records, std::function<bool(Record const&)> const& compare) {
    auto const iter = std::find_if(records.begin(), records.end(), compare);
    DCHECK(iter != records.end()) << "The supplied compare function does not evaluate true for any record in the "
                                  << typeid(Record).name() << " vector.";
    return std::distance(records.begin(), iter);
}

}  // namespace detail

PoseId GetMaxPoseId(PoseCollection const& coll) {
    PoseId result = 0;
    result = std::max_element(
                     coll.poses.begin(),
                     coll.poses.end(),
                     [](nie::io::PoseRecord const& a, nie::io::PoseRecord const& b) { return a.id < b.id; })
                     ->id;
    return result;
}

std::unordered_map<PoseId, std::size_t> CreatePoseMap(PoseCollection const& coll) {
    std::unordered_map<PoseId, std::size_t> map;
    for (std::size_t i = 0; i < coll.poses.size(); ++i) {
        auto result = map.emplace(coll.poses[i].id, i);
        DCHECK(result.second) << "Pose id " << coll.poses[i].id << " is used multiple times in the pose collection.";
    }
    return map;
}

std::vector<std::size_t> CreateFixedPoseMap(PoseCollection const& coll) {
    std::vector<std::size_t> map(coll.fixes.size());
    for (std::size_t i = 0; i < coll.fixes.size(); ++i) {
        auto const& fix = coll.fixes[i];
        map[i] = detail::FindIndex<nie::io::PoseRecord>(
                coll.poses, [&fix](nie::io::PoseRecord const& p) { return p.id == fix.id; });
    }
    return map;
}

std::vector<std::size_t> GetEdgesIndicesByCategory(
        std::vector<nie::io::PoseEdgeRecord> const& edges, nie::io::PoseEdgeRecord::Category category) {
    std::vector<std::size_t> indices;

    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (edges[i].category == category) {
            indices.push_back(i);
        }
    }
    return indices;
}

double GetMedianDistance(PoseCollection const& collection) {
    std::vector<double> distances(collection.poses.size() - 1);
    for (std::size_t i = 1; i < collection.poses.size(); ++i) {
        distances[i - 1] = collection.poses[i - 1]
                                   .isometry.TransformInverseLeft(collection.poses[i].isometry)
                                   .translation()
                                   .norm();
    }

    // strictly speaking, if we had an even number of elements, it would be the average of the middle two
    std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
    return *(distances.begin() + distances.size() / 2);
}

}  // namespace io

}  // namespace nie
