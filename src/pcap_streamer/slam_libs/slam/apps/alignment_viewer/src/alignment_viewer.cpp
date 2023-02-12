/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>
#include <nie/drawing/map_viewer.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>

#include "helper_registered_pair.hpp"
#include "pair_viewer.hpp"
#include "registered_pair.hpp"

// clang-format off
DEFINE_string(in_file_aa_bbox_abs_pose, "",
        "Location of the pose file containing the absolute axis-aligned (cartesian) bounding box poses.");
DEFINE_string(in_file_aa_bbox_bbox, "",
        "Location of the bbox file containing the axis-aligned (cartesian) bounding boxes.");
DEFINE_string(in_file_aa_bbox_iref, "",
        "Location of iref file that associates the axis-aligned (cartesian) bounding box poses with the las files.");
DEFINE_string(in_file_loops_pose, "",
        "Location of the pose file containing the bounding box loop closure edges.");
DEFINE_string(in_file_trace_abs_pose, "",
        "Location of the pose file containing the absolute traces.");
DEFINE_string(in_file_trace_rel_pose, "",
        "Location of the pose file containing the traces in the las file coordinate system. Optional, default is the"
        "same as the absolute one.");

DEFINE_string(jump_to_ids, "",
        "Two comma-separated id's from which to start viewing. [optional]");
DEFINE_bool(sort_on_error, false,
        "By default the pairs are viewed in order by id, setting this flag will show largest translation first. "
        "[optional]");
DEFINE_bool(sort_randomly, false,
        "By default the pairs are viewed in order by id, setting this flag will show the pairs randomly. [optional]");
DEFINE_double(filter_grid_size, 0.25,
        "Grid size use to filter the point cloud. Must be larger than 0.125 (default value). Recommended to use a "
        "value between 0.125 and 0.25. [optional]");
// clang-format on

DEFINE_validator(in_file_aa_bbox_abs_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_aa_bbox_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_aa_bbox_iref, nie::ValidateIsFile);
DEFINE_validator(in_file_loops_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_trace_abs_pose, nie::ValidateIsFile);

static constexpr auto kKeySymArrowLeft = "Left";
static constexpr auto kKeySymArrowRight = "Right";
static constexpr auto kKeySymBracketLeft = "bracketleft";
static constexpr auto kKeySymBracketRight = "bracketright";
static constexpr auto kKeySymBraceLeft = "braceleft";
static constexpr auto kKeySymBraceRight = "braceright";

std::unordered_set<std::size_t> GetBoundIndices(
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_id_to_bound_index_map,
        nie::RegisteredPair const& pair) {
    return {pose_id_to_bound_index_map.at(pair.id_a), pose_id_to_bound_index_map.at(pair.id_b)};
}

std::unordered_set<std::size_t> GetBoundIndices(
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_id_to_bound_index_map,
        std::vector<nie::RegisteredPair> const& pairs) {
    std::unordered_set<std::size_t> indices;
    std::for_each(pairs.cbegin(), pairs.cend(), [&pose_id_to_bound_index_map, &indices](auto const pair) {
        indices.insert(pose_id_to_bound_index_map.at(pair.id_a));
        indices.insert(pose_id_to_bound_index_map.at(pair.id_b));
    });
    return indices;
}

// TODO(jbr): Prevent trigger happy users from pressing keys while loading.
template <typename PointT>
void AddKeyboardListenerPairStep(
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_id_to_bound_index_map,
        PairViewer<PointT>* p_pair_viewer,
        nie::MapViewer* p_map_viewer,
        std::vector<nie::RegisteredPair>* p_pairs,
        std::size_t* p_stepper) {
    auto& pair_viewer = *p_pair_viewer;
    auto& map_viewer = *p_map_viewer;
    auto& pairs = *p_pairs;
    auto& stepper = *p_stepper;
    boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_event =
            [&pose_id_to_bound_index_map, &pair_viewer, &map_viewer, &pairs, &stepper](
                    const pcl::visualization::KeyboardEvent& event) -> void {
        // Don't want to do it twice (on up as well).
        if (event.keyDown()) {
            if (event.getKeySym() == kKeySymArrowLeft) {
                if (stepper > 0) {
                    stepper--;
                    pair_viewer.Update(pairs[stepper]);
                    map_viewer.Update(GetBoundIndices(pose_id_to_bound_index_map, pairs[stepper]));
                } else {
                    LOG(INFO) << "Reached begin of registrations.";
                }
                VLOG(6) << "keyboard_event(): kKeySymArrowLeft";
            } else if (event.getKeySym() == kKeySymArrowRight) {
                if (stepper < pairs.size() - 1) {
                    stepper++;
                    pair_viewer.Update(pairs[stepper]);
                    map_viewer.Update(GetBoundIndices(pose_id_to_bound_index_map, pairs[stepper]));
                } else {
                    LOG(INFO) << "Reached end of registrations.";
                }
                VLOG(6) << "keyboard_event(): kKeySymArrowRight";
            } else if (event.getKeySym() == kKeySymBracketLeft) {
                pair_viewer.visualize_registered_cloud() = !pair_viewer.visualize_registered_cloud();
                pair_viewer.Update();
                VLOG(6) << "keyboard_event(): kKeySymBracketLeft";
            } else if (event.getKeySym() == kKeySymBracketRight) {
                pair_viewer.visualize_unregistered_cloud() = !pair_viewer.visualize_unregistered_cloud();
                pair_viewer.Update();
                VLOG(6) << "keyboard_event(): kKeySymBracketRight";
            } else if (event.getKeySym() == kKeySymBraceLeft) {
                pair_viewer.visualize_registered_trace() = !pair_viewer.visualize_registered_trace();
                pair_viewer.Update();
                VLOG(6) << "keyboard_event(): kKeySymBraceLeft";
            } else if (event.getKeySym() == kKeySymBraceRight) {
                pair_viewer.visualize_unregistered_trace() = !pair_viewer.visualize_unregistered_trace();
                pair_viewer.Update();
                VLOG(6) << "keyboard_event(): kKeySymBraceRight";
            }
        }
    };
    pair_viewer.viewer().registerKeyboardCallback(keyboard_event);
}

template <typename PointT>
void Viewer() {
    CHECK(!(FLAGS_sort_on_error && FLAGS_sort_randomly))
            << "Only one setting, 'sort_on_error' or 'sort_randomly', can be selected.";

    // Read pose file with the bounding box centers
    LOG(INFO) << "Reading pose file with bbox poses: " << FLAGS_in_file_aa_bbox_abs_pose;
    auto const bbox_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_aa_bbox_abs_pose);

    // Read bbox file with the bounding boxes
    LOG(INFO) << "Reading bbox file: " << FLAGS_in_file_aa_bbox_bbox;
    auto const bbox_bbox = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_aa_bbox_bbox);

    // Read iref file relating the bounding box pose ids to the las file locations
    LOG(INFO) << "Reading iref file: " << FLAGS_in_file_aa_bbox_iref;
    auto const bbox_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_aa_bbox_iref);

    // Read pose file with the loop closure edges
    LOG(INFO) << "Reading pose file with the edges: " << FLAGS_in_file_loops_pose;
    auto const bbox_edges_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_loops_pose);

    // Read pose file with the relative and absolute trajectories
    LOG(INFO) << "Reading pose file with the absolute trace: " << FLAGS_in_file_trace_abs_pose;
    auto const trace_pose_abs = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_abs_pose);
    nie::io::PoseCollection trace_pose_rel_int{};
    if (FLAGS_in_file_trace_rel_pose.empty()) {
        LOG(INFO) << "Using the absolute trajectory also as the relative one.";
    } else {
        LOG(INFO) << "Reading pose file with the relative trace: " << FLAGS_in_file_trace_rel_pose;
        trace_pose_rel_int = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_rel_pose);
    }
    // To avoid copy
    nie::io::PoseCollection const& trace_pose_rel =
            FLAGS_in_file_trace_rel_pose.empty() ? trace_pose_abs : trace_pose_rel_int;

    std::unordered_map<nie::io::PoseId, std::size_t> pose_id_to_bound_index_map;
    std::vector<nie::PoseBbox> bounds;
    CreatePoseBboxMap(bbox_pose.poses, bbox_bbox.boxes, &pose_id_to_bound_index_map, &bounds);

    std::vector<nie::RegisteredPair> pairs = nie::GetLoopClosureCandidates(bbox_pose, bbox_iref, bbox_edges_pose);
    CHECK(!pairs.empty()) << "Could not find loop closure candidates.";

    // Sort the registered pairs
    if (FLAGS_sort_on_error) {
        // The first entries have the largest translation
        nie::SortRegisteredPairsByTranslationMagnitude(&pairs);
    } else if (FLAGS_sort_randomly) {
        nie::SortRegisteredPairsRandomly(&pairs);
    } else {
        // Default sorting on the id's
        std::sort(pairs.begin(), pairs.end(), [](nie::RegisteredPair const& a, nie::RegisteredPair const& b) -> bool {
            bool result = a.id_a < b.id_a;
            if (a.id_a == b.id_a) {
                result = a.id_b < b.id_b;
            }
            return result;
        });
    }

    std::size_t stepper = 0;
    // If the inspect ids flag is not empty, then jump to that pair
    if (not FLAGS_jump_to_ids.empty()) {
        std::vector<int> ids = nie::Split<int>(FLAGS_jump_to_ids);
        auto iter = std::find_if(pairs.cbegin(), pairs.cend(), [&ids](nie::RegisteredPair const& p) {
            return p.id_a == ids.front() and p.id_b == ids.back();
        });
        CHECK(iter != pairs.cend()) << "The supplied id's pair is not found in the matches.";
        stepper = static_cast<std::size_t>(std::distance(pairs.cbegin(), iter));
    }

    PairViewer<PointT> pair_viewer("alignment_viewer", trace_pose_rel.poses, FLAGS_filter_grid_size);

    nie::MapViewer map_viewer("cloud_map", bounds, trace_pose_abs, GetBoundIndices(pose_id_to_bound_index_map, pairs));
    map_viewer.Update(GetBoundIndices(pose_id_to_bound_index_map, pairs[stepper]));
    map_viewer.View();

    AddKeyboardListenerPairStep<PointT>(pose_id_to_bound_index_map, &pair_viewer, &map_viewer, &pairs, &stepper);
    pair_viewer.Update(pairs[stepper]);
    pair_viewer.View();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    Viewer<pcl::PointXYZI>();

    return 0;
}
