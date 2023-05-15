/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "filter_loops.hpp"

#include <nie/core/geometry/covariance.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/euclidean_distance.hpp>
#include <nie/lidar/geometry/helper_area.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>
#include <nie/lidar/slice.hpp>

#include <unordered_map>

namespace nie {

typedef std::unordered_map<io::PoseId, std::pair<std::size_t, std::size_t>> TraceRanges;

TraceRanges GetTracePerLas(
        io::PoseCollection const& bbox_loops,
        io::PoseCollection const& bbox_pose,
        io::InfoRefCollection const& bbox_iref,
        io::PoseCollection const& trajectory_pose,
        io::InfoRefCollection const& trajectory_iref) {
    auto const& traj_poses = trajectory_pose.poses;

    auto const bbox_pose_map = io::CreateRecordMap(bbox_pose.poses.begin(), bbox_pose.poses.end());
    auto const bbox_iref_map = io::CreateRecordMap(bbox_iref.info_refs.begin(), bbox_iref.info_refs.end());
    auto const traj_iref_map = io::CreateRecordMap(trajectory_iref.info_refs.begin(), trajectory_iref.info_refs.end());

    // Loop over the edges and create a list that relates the references bbox pose ids to a index marking the starting
    // point in the full trajectory that are included in the corresponding las file
    std::unordered_map<io::PoseId, std::size_t> bbox_search_index;
    auto const add_bbox_trace = [&traj_poses, &bbox_pose_map, &bbox_search_index](io::PoseId const id) {
        if (bbox_search_index.find(id) == bbox_search_index.end()) {
            // Each bbox pose points to the median timestamp of the trace in that box
            std::size_t const index =
                    std::lower_bound(traj_poses.cbegin(), traj_poses.cend(), bbox_pose_map.at(id).get().timestamp) -
                    traj_poses.cbegin();
            bbox_search_index.insert({id, index});
        }
    };
    for (auto const& e : bbox_loops.edges) {
        add_bbox_trace(e.id_begin);
        add_bbox_trace(e.id_end);
    }

    // Helper function that checks if the found index points to a pose that is included in las file and if so, if that
    // las file is the same as the provided one
    auto const traj_path_equals = [&traj_iref_map, &traj_poses](std::size_t const i, std::string const& las_path) {
        auto const pose_id = (traj_poses.cbegin() + i)->id;
        auto const iter = traj_iref_map.find(pose_id);
        if (iter == traj_iref_map.cend()) {
            return false;
        }
        return iter->second.get().path == las_path;
    };

    // Loop over the bboxes and create a map that contains whether the trace inside the bbox is long enough
    TraceRanges trace_per_las;
    for (auto& [bbox_pose_id, search_index] : bbox_search_index) {
        // Las file of the current bbox
        auto const las_path = bbox_iref_map.at(bbox_pose_id).get().path;

        // The indices that should mark the begin and (past the) end poses in the full trajectory for this las file
        std::size_t begin = search_index;
        std::size_t end = search_index;

        // Keep searching down the list until the trace refers to another LAS file
        while (begin > 0 && traj_path_equals(begin, las_path)) {
            begin--;
        }

        // Correct when the above loop went one iteration too far.
        if (!traj_path_equals(begin, las_path)) {
            begin++;
        }

        // Keep searching up the list until the trace refers to another LAS file
        while (end < traj_poses.size() && traj_path_equals(end, las_path)) {
            end++;
        }

        // Sanity checks to guarantee we found all possible poses to determine trace length.
        // Can't be tested directly through unit tests. Very useful in case someone accidentally changes something
        // resulting in a bug.
        // clang-format off
        DCHECK(begin == 0               || !traj_path_equals(begin - 1, las_path));
        DCHECK(                             traj_path_equals(begin    , las_path));
        DCHECK(                             traj_path_equals(end   - 1, las_path));
        DCHECK(end == traj_poses.size() || !traj_path_equals(end      , las_path));
        // clang-format on

        trace_per_las[bbox_pose_id] = std::make_pair(begin, end);
    }
    return trace_per_las;
}

std::unordered_map<io::PoseId, bool> GetBboxTraceHasMinLength(
        TraceRanges const& trace_ranges_per_las,
        io::PoseCollection const& trajectory_pose,
        double const minimum_trace_length) {
    auto const& traj_poses = trajectory_pose.poses;

    // Loop over the bboxes and create a map that contains whether the trace inside the bbox is long enough
    std::unordered_map<io::PoseId, bool> has_bbox_trace_min_length;
    for (auto const& [bbox_pose_id, range] : trace_ranges_per_las) {
        // The indices that should mark the begin and (past the) end poses in the full trajectory for this las file
        std::size_t const begin = range.first;
        std::size_t const end = range.second;

        auto const [it_slice_end, distance] = SliceByDistance(
                traj_poses.cbegin() + begin, traj_poses.cbegin() + end, minimum_trace_length, EuclideanDistance{});

        has_bbox_trace_min_length[bbox_pose_id] = it_slice_end != (traj_poses.cbegin() + end);
        VLOG(3) << "Trace id, accumulated distance: " << bbox_pose_id << ", " << distance;
    }
    return has_bbox_trace_min_length;
}

/// Creates a map that contains whether the average covariance scores of each BBox trace is below a maximum threshold.
std::unordered_map<io::PoseId, bool> GetBboxTracePositionIsReliable(
        TraceRanges const& trace_ranges_per_las,
        io::PoseCollection const& trajectory_pose,
        double const max_score_threshold) {
    auto const& traj_poses = trajectory_pose.poses;
    std::unordered_map<io::PoseId, bool> bbox_trace_is_reliable;

    for (auto const& [bbox_pose_id, range] : trace_ranges_per_las) {
        std::size_t const begin = range.first;
        std::size_t const end = range.second;

        double scores_sum = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
            auto const& current_pose = traj_poses[i];
            Eigen::Matrix<double, 6, 6> covariance;
            InformationToCovariance(current_pose.information, &covariance);
            // We use the Trace of the covariances matrix (only the position covariances) as a score metric, because we
            // are interested in measuring the total variation.
            // Note that a matrix's Trace is equivalent to the sum of its eigen-values.
            scores_sum += covariance.block<3, 3>(0, 0).trace();
        }

        std::size_t const count = end - begin;
        CHECK_GT(count, 0) << "Empty trace for bbox: " << bbox_pose_id;
        double const scores_avg = scores_sum / count;
        bool const is_reliable = scores_avg < max_score_threshold;
        bbox_trace_is_reliable[bbox_pose_id] = is_reliable;
        VLOG(3) << "trace_id positions_avg_score is_reliable: " << bbox_pose_id << " " << scores_avg << " "
                << std::boolalpha << is_reliable;
    }
    return bbox_trace_is_reliable;
}

// The loops are based on the cartesian bounding box poses in the sme system as the trace. Each of those poses has a
// corresponding oriented bounding box pose. Since they have a 1-1 correspondence we can use these for filtering the
// edges. Filtering based on:
//   * Area overlap.
//   * Area overlap ratio (a thin point cloud is hard to match).
//   * Length of trace going through each bbox.
io::PoseCollection FilterLoops(
        io::PoseCollection const& bbox_loops,
        io::PoseCollection const& bbox_pose,
        io::BboxCollection const& bbox_bbox,
        io::InfoRefCollection const& bbox_iref,
        io::PoseCollection const& trajectory_pose,
        io::InfoRefCollection const& trajectory_iref,
        double const intersection_area_threshold,
        double const intersection_ratio_threshold,
        double const minimum_trace_length,
        double const maximum_trace_cov_score) {
    auto bounds_map = io::CreatePoseBboxMap(bbox_pose.poses, bbox_bbox.boxes);
    auto const trace_per_las = GetTracePerLas(bbox_loops, bbox_pose, bbox_iref, trajectory_pose, trajectory_iref);
    auto const has_bbox_trace_min_length =
            GetBboxTraceHasMinLength(trace_per_las, trajectory_pose, minimum_trace_length);
    auto const bbox_trace_is_reliable =
            GetBboxTracePositionIsReliable(trace_per_las, trajectory_pose, maximum_trace_cov_score);

    io::PoseCollection bbox_loops_filtered;
    bbox_loops_filtered.header = bbox_loops.header;

    for (auto const& e : bbox_loops.edges) {
        PoseBbox const& bounds_b = bounds_map.at(e.id_begin);
        PoseBbox const& bounds_e = bounds_map.at(e.id_end);

        // Note that the filtering always takes place in the XY plane.
        if (!CheckIntersectionArea(bounds_b, bounds_e, intersection_area_threshold, intersection_ratio_threshold)) {
            continue;
        }

        if (!(has_bbox_trace_min_length.at(e.id_begin) && has_bbox_trace_min_length.at(e.id_end))) {
            continue;
        }

        // If both traces have reliable positioning, filter them out.
        if (bbox_trace_is_reliable.at(e.id_begin) && bbox_trace_is_reliable.at(e.id_end)) {
            continue;
        }

        bbox_loops_filtered.edges.push_back(e);
    }

    return bbox_loops_filtered;
}

}  // namespace nie
