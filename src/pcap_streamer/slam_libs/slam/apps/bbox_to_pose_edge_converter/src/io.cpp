/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "io.hpp"

#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>

std::unordered_set<nie::io::PoseId> ReadNodes(std::string const& filepath) {
    nie::CsvRecorderSafe<nie::io::PoseId> recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({filepath}, "id", &recorder);
    auto const records = recorder.ConvertToRecord();
    std::unordered_set<nie::io::PoseId> nodes;
    nodes.reserve(records.NumRows());
    for (std::size_t i = 0; i < records.NumRows(); ++i) {
        nodes.insert(std::get<0>(records.GetRow(i)));
    }
    return nodes;
}

std::unordered_map<nie::io::PoseId, std::vector<nie::io::PoseId>> GetTraceIdsByBoxId(
        std::unordered_set<nie::io::PoseId> const& linkable_pose_ids,
        nie::io::InfoRefCollection const& bbox_iref_coll,
        nie::io::InfoRefCollection const& trace_iref_coll) {
    std::unordered_map<std::string, nie::io::PoseId> bbox_id_by_path;
    std::unordered_map<nie::io::PoseId, std::vector<nie::io::PoseId>> trace_ids_by_box_id;
    for (auto const& bbox_iref : bbox_iref_coll.info_refs) {
        bbox_id_by_path[bbox_iref.path] = bbox_iref.id;
        // It may happen that there are no linkable pose ids for a particular bbox. In this case there should at least
        // be an empty set to refer to. If we don't create the empty set here, the loop below this one won't create it
        // when the box id is never referred to.
        trace_ids_by_box_id[bbox_iref.id] = std::vector<nie::io::PoseId>();
    }

    for (auto const& trace_iref : trace_iref_coll.info_refs) {
        if (linkable_pose_ids.find(trace_iref.id) != linkable_pose_ids.end()) {
            auto sr = bbox_id_by_path.find(trace_iref.path);
            if (sr != bbox_id_by_path.end()) {
                trace_ids_by_box_id[sr->second].push_back(trace_iref.id);
            }
        }
    }

    return trace_ids_by_box_id;
}

void FilterPoseIds(
        nie::io::PoseEdgeRecord const& bbox_edge,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& aa_bbox_pose_map,
        std::unordered_map<nie::io::PoseId, nie::PoseBbox> const& oa_bounds_map,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& trace_pose_map,
        std::vector<nie::io::PoseId>* p_trace_b,
        std::vector<nie::io::PoseId>* p_trace_e) {
    std::vector<nie::io::PoseId>& trace_b = *p_trace_b;
    std::vector<nie::io::PoseId>& trace_e = *p_trace_e;

    auto const& aa_pose_b = aa_bbox_pose_map.at(bbox_edge.id_begin).get();
    auto const& aa_pose_e = aa_bbox_pose_map.at(bbox_edge.id_end).get();
    auto oa_bbox_b = oa_bounds_map.at(bbox_edge.id_begin);
    auto oa_bbox_e = oa_bounds_map.at(bbox_edge.id_end);

    // Since we work using the original LOAM origins, there is no common world (like UTM or gauss kruger) to use for
    // transforming poses. Each LOAM trace lives in its own world. We use the edge between two axis aligned bounding
    // boxes to bring everything into a common reference frame.

    // So we bring the LOAM poses to the bbox of the same loam world first, then use the edge to transfer
    // the poses to another bbox of a different loam world. We then transform from axis aligned bbox to oriented axis
    // bbox. At the end we take the inverse to use that point transformation as an origin.
    oa_bbox_b.origin() =
            (oa_bbox_b.origin().Inversed() * aa_pose_b.isometry * bbox_edge.isometry * aa_pose_e.isometry.Inversed())
                    .Inversed();
    oa_bbox_e.origin() = (oa_bbox_e.origin().Inversed() * aa_pose_e.isometry * bbox_edge.isometry.Inversed() *
                          aa_pose_b.isometry.Inversed())
                                 .Inversed();

    VLOG(3) << "Trace sizes before filtering of begin and end: " << trace_b.size() << ", " << trace_e.size()
            << std::endl;

    // Remove poses from trace "begin" that are not in bbox "end"
    trace_b.erase(
            std::remove_if(
                    trace_b.begin(),
                    trace_b.end(),
                    [&oa_bbox_e, &trace_pose_map](nie::io::PoseId const id) -> bool {
                        return !oa_bbox_e.Contains(trace_pose_map.at(id).get().isometry.translation());
                    }),
            trace_b.end());

    // Remove poses from trace "end" that are not in bbox "begin"
    trace_e.erase(
            std::remove_if(
                    trace_e.begin(),
                    trace_e.end(),
                    [&oa_bbox_b, &trace_pose_map](nie::io::PoseId const id) -> bool {
                        return !oa_bbox_b.Contains(trace_pose_map.at(id).get().isometry.translation());
                    }),
            trace_e.end());

    VLOG(3) << "Trace sizes after filtering of begin and end: " << trace_b.size() << ", " << trace_e.size()
            << std::endl;
}
