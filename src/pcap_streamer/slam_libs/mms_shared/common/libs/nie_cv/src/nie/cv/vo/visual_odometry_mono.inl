/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "ba_problem.hpp"
#include "back_projection.hpp"

namespace nie {

template <VisualOdometryMono::Handle handle>
void VisualOdometryMono::CalculateBackProjections(
        std::vector<Isometry3md> const& poses,
        std::vector<KeypointVector> keypoint_vectors,
        std::vector<Eigen::Vector3d> const& objects) {
    assert(objects.size() == keypoint_vectors.size());
    for (auto const& keypoints [[maybe_unused]] : keypoint_vectors) {
        assert(poses.size() == keypoints.size());
    }

    if (!HasCallback<handle>()) {
        return;
    }

    std::vector<KeypointVector> orig_kpnts, bp_kpnts;

    for (std::size_t frame_index = 0; frame_index < poses.size(); ++frame_index) {
        std::vector<Eigen::Vector3d> selected_objects;
        selected_objects.reserve(objects.size());

        orig_kpnts.emplace_back();
        orig_kpnts.back().reserve(objects.size());
        for (std::size_t object_index = 0; object_index < objects.size(); ++object_index) {
            Keypoint const& keypoint = keypoint_vectors[object_index][frame_index];
            if (keypoint != kDefaultKeypoint) {
                selected_objects.emplace_back(objects[object_index]);
                orig_kpnts.back().emplace_back(keypoint);
            }
        }

        bp_kpnts.emplace_back();
        bp_kpnts.back().reserve(selected_objects.size());
        CalculateBackProjection(selected_objects, poses[frame_index], parameters_.K, &bp_kpnts.back());
    }

    prev_ids_.Reorder();
    Callback<handle>(prev_ids_.Data(), orig_kpnts, bp_kpnts);
}

}  // namespace nie
