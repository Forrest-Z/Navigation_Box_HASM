/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "concat.hpp"

#include <unordered_map>

#include "tool/io.hpp"

[[nodiscard]] nie::io::PoseCollection Concat(std::vector<nie::io::PoseCollection> const& collections) {
    nie::io::PoseCollection concat;
    concat.header = collections[0].header;

    nie::io::PoseId id = 0;

    for (auto const& c : collections) {
        std::unordered_map<nie::io::PoseId, nie::io::PoseId> old_to_new;

        for (auto const& p : c.poses) {
            old_to_new[p.id] = id;
            concat.poses.push_back(p);
            concat.poses.back().id = id;
            id++;
        }

        for (auto const& e : c.edges) {
            concat.edges.push_back(e);
            concat.edges.back().id_begin = old_to_new[e.id_begin];
            concat.edges.back().id_end = old_to_new[e.id_end];
        }

        for (auto const& f : c.fixes) {
            concat.fixes.push_back({old_to_new[f.id]});
        }
    }

    return concat;
}

template <typename CollectionType, typename MemFn>
[[nodiscard]] CollectionType Concat(std::vector<CollectionType> const& collections, MemFn mem_fn) {
    CollectionType concat;
    concat.header = collections[0].header;

    nie::io::PoseId id = 0;

    for (auto const& c : collections) {
        for (auto const& i : mem_fn(c)) {
            mem_fn(concat).push_back(i);
            mem_fn(concat).back().id = id;
            id++;
        }
    }

    return concat;
}

void Concat() {
    std::vector<nie::io::PoseCollection> pose_data;
    ReadData(&pose_data);
    CHECK(pose_data.size() > 1) << "Nothing to concat.";

    CheckAllHeadersAreIdentical(pose_data);

    std::vector<nie::io::InfoRefCollection> iref_data;
    if (ReadData(&iref_data)) {
        CHECK(iref_data.size() == pose_data.size()) << "Expected as many .iref files as .pose files (or none).";
        for (std::size_t i = 0; i < pose_data.size(); ++i) {
            CHECK(pose_data[i].poses.size() == iref_data[i].info_refs.size())
                    << "Implementation of concat assumes an iref record for every pose.";
        }
    }

    std::vector<nie::io::BboxCollection> bbox_data;
    if (ReadData(&bbox_data)) {
        CHECK(bbox_data.size() == pose_data.size()) << "Expected as many .bbox files as .pose files (or none).";
        for (std::size_t i = 0; i < pose_data.size(); ++i) {
            CHECK(pose_data[i].poses.size() == bbox_data[i].boxes.size())
                    << "Implementation of concat assumes a bbox record for every pose.";
        }
    }

    WriteData(Concat(pose_data));

    if (!iref_data.empty()) {
        WriteData(Concat(iref_data, std::mem_fn(&nie::io::InfoRefCollection::info_refs)));
    }

    if (!bbox_data.empty()) {
        WriteData(Concat(bbox_data, std::mem_fn(&nie::io::BboxCollection::boxes)));
    }
}
