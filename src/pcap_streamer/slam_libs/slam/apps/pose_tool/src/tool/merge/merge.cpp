/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "merge.hpp"

#include <map>
#include <numeric>

#include "tool/io.hpp"

//
// Functions related to the merging of headers
//

using EigenMatrix6d = Eigen::Matrix<double, 6, 6>;

bool GetCommonPoseInfo(std::vector<nie::io::PoseCollection> const& collections, EigenMatrix6d* info) {
    bool info_found = false;
    EigenMatrix6d tmp_info;
    bool all_info_same = true;

    for (auto const& c : collections) {
        if (c.header.Has(nie::io::PoseHeader::kHasPoseInformation)) {
            if (info_found) {
                all_info_same = c.header.pose_information == tmp_info;
            } else {
                info_found = true;
                tmp_info = c.header.pose_information;
            }
            if (!all_info_same) {
                return false;
            }
        }
    }

    CHECK(info_found);
    if (all_info_same) {
        *info = tmp_info;
    }
    return all_info_same;
}

bool GetCommonEdgeInfo(std::vector<nie::io::PoseCollection> const& collections, EigenMatrix6d* info) {
    bool info_found = false;
    EigenMatrix6d tmp_info;
    bool all_info_same = true;

    for (auto const& c : collections) {
        if (c.header.Has(nie::io::PoseHeader::kHasEdgeInformation)) {
            if (info_found) {
                all_info_same = c.header.edge_information == tmp_info;
            } else {
                info_found = true;
                tmp_info = c.header.edge_information;
            }
            if (!all_info_same) {
                return false;
            }
        }
    }

    CHECK(info_found);
    if (all_info_same) {
        *info = tmp_info;
    }
    return all_info_same;
}

nie::io::PoseHeader MergeOrFatalHeaders(std::vector<nie::io::PoseCollection> const& collections) {
    // First do the simple checks
    auto const& h = collections.front().header;
    CHECK(std::all_of(collections.cbegin(), collections.cend(), [h](auto const& c) {
        return c.header.version == h.version;
    })) << "Versions do not match.";
    CHECK(std::all_of(collections.cbegin(), collections.cend(), [h](auto const& c) {
        return nie::io::EqualAuthority(c.header, h);
    })) << "Authorities do not match.";
    CHECK(std::all_of(collections.cbegin(), collections.cend(), [h](auto const& c) {
        return c.header.Has(nie::io::PoseHeader::kHasTimestampPerRecord) ==
               h.Has(nie::io::PoseHeader::kHasTimestampPerRecord);
    })) << "Timestamp information does not match.";

    // Now start doing the harder checks

    // Determine if there will be any information given for poses and edges
    bool const any_pose_info =
            std::any_of(collections.cbegin(), collections.cend(), [](nie::io::PoseCollection const& c) {
                return (!c.poses.empty()) && c.header.HasAnyPoseInformation();
            });
    bool const any_edge_info =
            std::any_of(collections.cbegin(), collections.cend(), [](nie::io::PoseCollection const& c) {
                return (!c.edges.empty()) && c.header.HasAnyEdgeInformation();
            });
    VLOG(3) << "Is there a collection with poses and any pose information? " << (any_pose_info ? "yes" : "no");
    VLOG(3) << "Is there a collection with edges and any edge information? " << (any_edge_info ? "yes" : "no");

    // Check for all collections that when there is a collection having any pose information, then each collection
    // should be no poses or it should be have pose information. Similar logic applies to the edges.
    if (any_pose_info) {
        CHECK(std::all_of(collections.cbegin(), collections.cend(), [](auto const& c) {
            return c.poses.empty() | c.header.HasAnyPoseInformation();
        })) << "Some collections have pose information, so all collections with poses should have it.";
    }
    if (any_edge_info) {
        CHECK(std::all_of(collections.cbegin(), collections.cend(), [](auto const& c) {
            return c.edges.empty() | c.header.HasAnyEdgeInformation();
        })) << "Some collections have edge information, so all collections with edges should have it.";
    }

    // At this stage it is known that the header can be combined.
    // Depending on which information is given, the flags are set.

    nie::io::PoseHeader result = h;
    result.Unset(nie::io::PoseHeader::kHasPoseInformation);
    result.Unset(nie::io::PoseHeader::kHasPoseInformationPerRecord);
    if (any_pose_info) {
        // Determine if there is any information given per record
        bool any_pose_info_per_record =
                std::any_of(collections.cbegin(), collections.cend(), [](nie::io::PoseCollection const& c) {
                    return c.header.Has(nie::io::PoseHeader::kHasPoseInformationPerRecord);
                });
        // If the information is not the same for all files, then still make it per record
        if (!any_pose_info_per_record) {
            bool common_pose_info = GetCommonPoseInfo(collections, &result.pose_information);
            if (!common_pose_info) {
                any_pose_info_per_record = true;
            }
        }

        if (any_pose_info_per_record) {
            result.Set(nie::io::PoseHeader::kHasPoseInformationPerRecord);
        } else {
            result.Set(nie::io::PoseHeader::kHasPoseInformation);
        }
    }
    result.Unset(nie::io::PoseHeader::kHasEdgeInformation);
    result.Unset(nie::io::PoseHeader::kHasEdgeInformationPerRecord);
    if (any_edge_info) {
        // Determine if there is any information given per record
        bool any_edge_info_per_record =
                std::any_of(collections.cbegin(), collections.cend(), [](nie::io::PoseCollection const& c) {
                    return c.header.Has(nie::io::PoseHeader::kHasEdgeInformationPerRecord);
                });
        // If the information is not the same for all files, then still make it per record
        if (!any_edge_info_per_record) {
            bool common_edge_info = GetCommonEdgeInfo(collections, &result.edge_information);
            if (!common_edge_info) {
                any_edge_info_per_record = true;
            }
        }

        if (any_edge_info_per_record) {
            result.Set(nie::io::PoseHeader::kHasEdgeInformationPerRecord);
        } else {
            result.Set(nie::io::PoseHeader::kHasEdgeInformation);
        }
    }

    return result;
}

nie::io::InfoRefHeader MergeOrFatalHeaders(std::vector<nie::io::InfoRefCollection> const& collections) {
    CheckAllHeadersAreIdentical(collections);
    return collections.front().header;
}

//
// Functions related to the merging of collections
//

void Merge(std::vector<nie::io::PoseCollection> const& pose_data, nie::io::PoseCollection* p_collection) {
    nie::io::PoseCollection& collection = *p_collection;

    std::map<nie::io::PoseId, std::size_t> pose_id_to_index;

    // First we add the poses in case someone cheats and uses a .pose file that only contains edges.
    for (auto const& c : pose_data) {
        // Note: The goal is to remove duplicates between files, but it would also do so within a single one. This is
        // not checked.
        for (auto const& p : c.poses) {
            auto it = pose_id_to_index.find(p.id);
            if (it == pose_id_to_index.end()) {
                pose_id_to_index[p.id] = collection.poses.size();
                collection.poses.push_back(p);
                if (collection.header.Has(nie::io::PoseHeader::kHasPoseInformationPerRecord) &&
                    c.header.Has(nie::io::PoseHeader::kHasPoseInformation)) {
                    collection.poses.back().information = c.header.pose_information;
                }
            } else {
                CHECK(nie::io::Equals(collection.poses[it->second], p, collection.header));
            }
        }
    }

    std::map<std::pair<nie::io::PoseId, nie::io::PoseId>, std::size_t> edge_id_to_index;

    for (auto const& c : pose_data) {
        for (auto const& e : c.edges) {
            auto key = std::make_pair(e.id_begin, e.id_end);
            auto it = edge_id_to_index.find(key);
            if (it == edge_id_to_index.end()) {
                edge_id_to_index[key] = collection.edges.size();
                collection.edges.push_back(e);
                if (collection.header.Has(nie::io::PoseHeader::kHasEdgeInformationPerRecord) &&
                    c.header.Has(nie::io::PoseHeader::kHasEdgeInformation)) {
                    collection.edges.back().information = c.header.edge_information;
                }
            } else {
                CHECK(nie::io::Equals(collection.edges[it->second], e, collection.header));
            }
        }
    }

    std::map<nie::io::PoseId, std::size_t> fix_id_to_index;

    for (auto const& c : pose_data) {
        for (auto const& f : c.fixes) {
            auto it = fix_id_to_index.find(f.id);
            if (it == fix_id_to_index.end()) {
                fix_id_to_index[f.id] = collection.fixes.size();
                collection.fixes.push_back(f);
            } else {
                CHECK(nie::io::Equals(collection.fixes[it->second], f));
            }
        }
    }
}

void Merge(std::vector<nie::io::InfoRefCollection> const& iref_data, nie::io::InfoRefCollection* p_collection) {
    nie::io::InfoRefCollection& collection = *p_collection;

    std::map<std::pair<nie::io::PoseId, decltype(nie::io::InfoRefRecord::frame_id)>, std::size_t> ids_to_index;

    for (auto const& c : iref_data) {
        // Note: The goal is to remove duplicates between files, but it would also do so within a single one. This is
        // not checked.
        for (auto const& i : c.info_refs) {
            auto const key = std::make_pair(i.id, i.frame_id);
            auto it = ids_to_index.find(key);
            if (it == ids_to_index.end()) {
                ids_to_index[key] = collection.info_refs.size();
                collection.info_refs.push_back(i);
            } else {
                CHECK(nie::io::Equals(collection.info_refs[it->second], i));
            }
        }
    }
}

template <typename Collection>
void MergeCollections() {
    std::vector<Collection> data;
    ReadData(&data);
    if (data.empty()) {
        return;
    }

    CHECK(data.size() > 1) << "Nothing to merge, only 1 " << nie::io::graph::Extension<Collection>()
                           << " file supplied.";

    Collection result;
    result.header = MergeOrFatalHeaders(data);
    Merge(data, &result);
    WriteData(result);
}

void Merge() {
    MergeCollections<nie::io::PoseCollection>();
    MergeCollections<nie::io::InfoRefCollection>();
}
