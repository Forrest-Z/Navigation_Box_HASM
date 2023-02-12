/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "helper_registered_pair.hpp"

#include <random>
#include <unordered_set>

#include <nie/core/geometry/rotation.hpp>

namespace nie {

std::pair<double, double> CalculateCorrection(RegisteredPair const& pair) {
    auto const T_corr = pair.T_ab_icp * pair.bbox_pose_abs_b.Inversed() * pair.bbox_pose_abs_a;
    return {T_corr.translation().norm(), nie::Rad2Deg(Eigen::AngleAxisd(T_corr.rotation().normalized()).angle())};
}

void SortRegisteredPairsByTranslationMagnitude(std::vector<RegisteredPair>* pairs) {
    // Sort the matched pairs based on the translation difference, largest first
    std::sort(pairs->begin(), pairs->end(), [](RegisteredPair const& a, RegisteredPair const& b) {
        return CalculateCorrection(a).first > CalculateCorrection(b).first;
    });
}

void SortRegisteredPairsRandomly(std::vector<RegisteredPair>* pairs) {
    auto const seed = std::random_device{}();
    std::default_random_engine rng{seed};
    std::shuffle(pairs->begin(), pairs->end(), rng);
}

std::vector<RegisteredPair> GetLoopClosureCandidates(
        io::PoseCollection const& bbox_pose,
        io::InfoRefCollection const& bbox_iref,
        io::PoseCollection const& edges_pose) {
    DVLOG(3) << "Found " << bbox_pose.poses.size() << " poses in bounding box pose file.";
    DVLOG(3) << "Found " << edges_pose.edges.size() << " edges in loop closure edges pose file.";

    std::vector<RegisteredPair> pairs{};

    // Create mapping of bounding box poses
    std::unordered_map<io::PoseId, std::reference_wrapper<io::PoseRecord const>> const pose_by_id =
            io::CreateRecordMap(bbox_pose.poses.cbegin(), bbox_pose.poses.cend());

    // Create mapping of LAS files
    std::unordered_map<io::PoseId, std::reference_wrapper<io::InfoRefRecord const>> const info_ref_by_id =
            io::CreateRecordMap(bbox_iref.info_refs.cbegin(), bbox_iref.info_refs.cend());

    // Get all edges of loop closure matches
    DVLOG(3) << "Creating filter pose edge map...";
    std::vector<std::size_t> const edge_indices =
            io::GetEdgesIndicesByCategory(edges_pose.edges, io::PoseEdgeRecord::Category::kLoop);
    DVLOG(3) << "Found " << edge_indices.size() << " loop closure edges.";

    for (size_t const& idx : edge_indices) {
        DCHECK(idx < edges_pose.edges.size()) << idx << " is out of range for pose edges.";
        io::PoseEdgeRecord const& constraint = edges_pose.edges[idx];
        io::PoseId const& id_a = constraint.id_begin;
        io::PoseId const& id_b = constraint.id_end;

        auto const pose_a_it = pose_by_id.find(id_a);
        auto const pose_b_it = pose_by_id.find(id_b);

        CHECK(pose_a_it != pose_by_id.cend()) << "pose record with id_a " << id_a << " was not found. "
                                              << "Mismatch between pose file and LAS directory.";
        CHECK(pose_b_it != pose_by_id.cend()) << "pose record with id_b " << id_b << " was not found."
                                              << "Mismatch between pose file and LAS directory.";

        auto const file_a_it = info_ref_by_id.find(id_a);
        auto const file_b_it = info_ref_by_id.find(id_b);

        CHECK(file_a_it != info_ref_by_id.cend()) << "file id_a " << id_a << " was not found. "
                                                  << "Mismatch between pose file and LAS directory.";
        CHECK(file_b_it != info_ref_by_id.cend()) << "file id_b " << id_b << " was not found."
                                                  << "Mismatch between pose file and LAS directory.";

        pairs.push_back(
                {id_a,
                 id_b,
                 pose_a_it->second.get().timestamp,
                 pose_b_it->second.get().timestamp,
                 pose_a_it->second.get().isometry,
                 pose_b_it->second.get().isometry,
                 constraint.isometry,
                 file_a_it->second.get().path,
                 file_b_it->second.get().path});
    }
    return pairs;
}

}  // namespace nie
