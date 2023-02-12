/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>
#include <nie/core/work_pool.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>
#include <nie/lidar/io/las_reader.hpp>

#include "debug_io.hpp"
#include "loop_closure_matcher.hpp"
#include "registration_result.hpp"

// clang-format off
DEFINE_string(in_file_pose_trace, "",
        "Location of the .pose file with the not-offset loam trace.");
DEFINE_string(in_file_pose_edges, "",
        "Location of the .pose file with the candidate loop closure edges.");
DEFINE_string(in_file_bbox_bbox, "",
        "Location of the oriented not-offset .bbox file with the bounding box information.");
DEFINE_string(in_file_pose_bbox, "",
        "Location of the oriented not-offset .pose file with the bounding box origin.");
DEFINE_string(in_file_iref_bbox, "",
        "Location of the oriented not-offset .iref file relating the bounding boxes to the las files.");
DEFINE_string(out_file_pose_edges,"",
        "Location of the .pose file to which the found loop closure matches will be written with transformations "
        "between the trajectory poses.");
// clang-format on

DEFINE_validator(in_file_pose_trace, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_edges, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_iref_bbox, nie::ValidateIsFile);
DEFINE_validator(out_file_pose_edges, nie::ValidateParentDirExists);

// DEBUGGING AND RELATED FLAGS
DEFINE_bool(debug, false, "Outputs extra debug .LAS files and a loops only g2o file.");
DEFINE_string(out_dir_debug, "", "Directory to write the debug results.");
DEFINE_bool(dry_run, false, "The application ONLY determines loop candidates without performing registration.");
DEFINE_int64(num_compares, -1, "Number of loop candidates to process. Default -1 for matching all candidates.");

// Pyramid scaled parameters. It is expected that after each tier the error should
// fall within the max_correspondence_distance and ransac_outlier_rejection_threshold
// of the next tier.
// The goal is simply to reduce both to be close to the actual transformation error.
// This can avoid match distractions and allows for refinement.
// RANSAC is otherwise used for a possible random restart (getting out of a local
// minimum), to remove border effects and other spurious "mismatches". It will try to
// get transformation agreement between matches (match being the closest point, not the
// actual correct match). This speeds up the convergence quite a bit.
// Last step not part of the pyramid but purely for refinement. Before refinement
// the error shouldn't be much larger than the thresholds.
template <typename PointT>
std::vector<typename nie::CloudMatcher<PointT>::Parameters> GetMatchParameters() {
    std::vector<typename nie::CloudMatcher<PointT>::Parameters> matcher_parameters;
    auto parameters = typename nie::CloudMatcher<PointT>::Parameters();

    parameters.transformation_eps = 1.e-9;
    parameters.euclidean_fitness_eps = -1.0;
    parameters.max_iterations = 30;
    parameters.ransac_max_iterations = 15;

    // Alignment step 1
    parameters.max_correspondence_distance = 10.0;
    parameters.ransac_outlier_rejection_threshold = 5.0;
    matcher_parameters.push_back(parameters);

    // Alignment step 2
    parameters.max_correspondence_distance = 5.0;
    parameters.ransac_outlier_rejection_threshold = 2.5;
    matcher_parameters.push_back(parameters);

    // Alignment step 3
    parameters.max_correspondence_distance = 2.5;
    parameters.ransac_outlier_rejection_threshold = 1.25;
    parameters.max_iterations = 20;
    matcher_parameters.push_back(parameters);

    // Alignment step 4
    parameters.max_correspondence_distance = 1.25;
    parameters.ransac_outlier_rejection_threshold = 0.625;
    matcher_parameters.push_back(parameters);

    return matcher_parameters;
}

// Define the size of the voxel grid subsampling filter
template <typename PointT>
typename nie::CloudFilter<PointT>::Parameters GetFilterParameters() {
    return typename nie::CloudFilter<PointT>::Parameters(0.15);
}

Candidates CollectCandidates(
        nie::io::PoseCollection const& edges,
        nie::io::BboxCollection const& bbox,
        nie::io::PoseCollection const& pose,
        nie::io::InfoRefCollection const& iref) {
    std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const pose_map =
            nie::io::CreateRecordMap(pose.poses.cbegin(), pose.poses.cend());
    std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::InfoRefRecord const>> const info_map =
            nie::io::CreateRecordMap(iref.info_refs.cbegin(), iref.info_refs.cend());
    std::unordered_map<nie::io::PoseId, nie::PoseBbox> const bounds_map =
            nie::io::CreatePoseBboxMap(pose.poses, bbox.boxes);

    Candidates candidates(edges.edges.size());
    std::transform(
            edges.edges.cbegin(),
            edges.edges.cend(),
            candidates.begin(),
            [&pose_map, &info_map, &bounds_map](nie::io::PoseEdgeRecord const& e) -> Candidate {
                return {e.id_begin,
                        e.id_end,
                        nie::ToGPSWeekTime(pose_map.at(e.id_begin).get().timestamp).week,
                        nie::ToGPSWeekTime(pose_map.at(e.id_end).get().timestamp).week,
                        info_map.at(e.id_begin).get().path,
                        info_map.at(e.id_end).get().path,
                        bounds_map.at(e.id_begin),
                        bounds_map.at(e.id_end),
                        e.isometry};
            });
    return candidates;
}

void WriteRegistrationResults(
        std::string const& out_file_pose_edges,
        nie::io::PoseHeader const& header,
        std::vector<RegistrationResult> const& registration_results) {
    std::vector<nie::io::PoseEdgeRecord> edges;
    std::transform(
            registration_results.cbegin(),
            registration_results.cend(),
            std::back_inserter(edges),
            [](RegistrationResult const& r) -> nie::io::PoseEdgeRecord {
                return nie::io::PoseEdgeRecord{
                        r.ids.first, r.ids.second, nie::io::PoseEdgeRecord::Category::kLoop, r.T, {}};
            });

    LOG(INFO) << "Writing the edges found to '" << out_file_pose_edges << "'";
    nie::io::Write(nie::io::PoseCollection{header, {}, edges, {}}, out_file_pose_edges);
}

// Actual pipeline to be executed
template <typename PointT>
void PipeLine() {
    LOG(INFO) << "Reading the candidate edges from '" << FLAGS_in_file_pose_edges << "'";
    auto pose_collection_edges = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_edges);
    if (pose_collection_edges.edges.empty()) {
        LOG(WARNING) << "No loop candidates supplied, so no loop closing to be done.";
        WriteRegistrationResults(FLAGS_out_file_pose_edges, pose_collection_edges.header, {});
        return;
    }

    LOG(INFO) << "Reading the loam trace from '" << FLAGS_in_file_pose_trace << "'";
    auto pose_collection_trace = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_trace);
    LOG(INFO) << "Reading the bounding boxes from '" << FLAGS_in_file_bbox_bbox << "'";
    auto bbox_bbox_collection = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_bbox_bbox);
    LOG(INFO) << "Reading the bounding box poses from '" << FLAGS_in_file_pose_bbox << "'";
    auto bbox_pose_collection = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_bbox);
    LOG(INFO) << "Reading the bounding box irefs from '" << FLAGS_in_file_iref_bbox << "'";
    auto bbox_iref_collection = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_iref_bbox);

    Candidates candidates =
            CollectCandidates(pose_collection_edges, bbox_bbox_collection, bbox_pose_collection, bbox_iref_collection);

    if (FLAGS_num_compares > -1) {
        std::size_t const new_size = std::min(candidates.size(), static_cast<std::size_t>(FLAGS_num_compares));
        LOG(INFO) << "Only processing first " << new_size << " loop candidates.";
        candidates.resize(new_size);
    }

    CHECK(!FLAGS_debug || nie::ValidateIsDirectory("out_dir_debug", FLAGS_out_dir_debug))
            << "The flag out_dir_debug is required and should be an existing directory when running in debug mode.";

    // Full transformation bringing points from system of cloud b to system of cloud a, including the ICP correction
    nie::mt::MtVector<RegistrationResult> registration_results;

    nie::DebugIo debug_io(FLAGS_out_dir_debug, FLAGS_debug);

    if (!candidates.empty()) {
        // Create the loop closure matcher
        LoopClosureMatcher<PointT> loop_closure_matcher(
                GetFilterParameters<PointT>(),
                GetMatchParameters<PointT>(),
                pose_collection_trace.poses,
                FLAGS_dry_run,
                &registration_results,
                &debug_io);

        // Determine the number of cores to parallelize over
        unsigned core_count = std::thread::hardware_concurrency() / 3;

        // Use scope to ensure that work added to the work pool is complete on scope closure
        {
            // Create a work pool with the same number of threads as cores
            nie::WorkPool<Candidate> pool_loop_closer(core_count, core_count * 2, std::move(loop_closure_matcher));

            // Loop over all candidates
            for (Candidate candidate : candidates) {
                // Add candidate loop closures to the work queue to process in parallel on available threads
                auto work = std::make_unique<Candidate>(std::move(candidate));
                pool_loop_closer.queue().BlockingPushBack(std::move(work));
            }
        }
    }

    WriteRegistrationResults(FLAGS_out_file_pose_edges, pose_collection_edges.header, registration_results.vector());
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    if (VLOG_IS_ON(6)) {
        pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
    }

    PipeLine<pcl::PointXYZI>();

    return 0;
}
