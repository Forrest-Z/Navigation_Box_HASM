/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "visual_odometry_object_keypoints.hpp"

#include <glog/logging.h>

namespace nie {

namespace detail {

std::pair<VisualOdometryObjectKeypoints::ObjectId, bool> VisualOdometryObjectKeypoints::GetObjectId(
    FrameId const& id, std::size_t const& match_index) {
    // Look up object id or generate a new one
    std::pair<Iter, bool> insert_result = map_object_id_.insert({{id, match_index}, 0});
    ObjectId& object_id = insert_result.first->second;
    if (insert_result.second) {
        object_id = AddNewObject();
    }
    return {object_id, insert_result.second};
}

void VisualOdometryObjectKeypoints::SetObjectId(
    ObjectId const& object_id, FrameId const& id, std::size_t const& match_index) {
    // Create/update the current object in any case
    map_object_id_[{id, match_index}] = object_id;
}

bool VisualOdometryObjectKeypoints::ValidObjectId(ObjectId const& object_id) const {
    return object_ids_.count(object_id) != 0;
}

void VisualOdometryObjectKeypoints::PrepareKeypoints(FrameId const& id) {
    for (auto& keypoint : all_keypoints_) {
        keypoint.emplace_back(id, default_keypoint_);
    }
}

void VisualOdometryObjectKeypoints::AddKeypoint(
    ObjectId const& object_id, FrameId const& id, KeypointType const& keypoint) {
    CheckObjectId(object_id);
    all_keypoints_[object_id].emplace_back(id, keypoint);
}

void VisualOdometryObjectKeypoints::SetKeypoint(
    ObjectId const& object_id, FrameId const& id, KeypointType const& keypoint) {
    CheckObjectId(object_id);
    std::vector<std::pair<FrameId, KeypointType>>& keypoints = all_keypoints_[object_id];
    if (keypoints.empty()) {
        keypoints.emplace_back(id, keypoint);
    } else {
        std::pair<FrameId, KeypointType>& pair = keypoints.back();
        assert(pair.first == id);
        pair.second = keypoint;
    }
}

void VisualOdometryObjectKeypoints::SetObject(ObjectId const& object_id, Eigen::Vector3d object) {
    CheckObjectId(object_id);
    all_objects_[object_id] = std::move(object);
}

std::vector<VisualOdometryObjectKeypoints::ObjectId> VisualOdometryObjectKeypoints::GetObjectIdsToWrite(
    std::size_t const overlap) const {
    std::vector<ObjectId> result;
    for (ObjectId const& object_id : object_ids_) {
        auto const& keypoints = all_keypoints_[object_id];
        if (keypoints.size() > overlap && (keypoints.end() - overlap)->second == default_keypoint_) {
            result.push_back(object_id);
        }
    }
    return result;
}

void VisualOdometryObjectKeypoints::GetBaData(
    std::size_t const& size,
    std::vector<ObjectId>* p_object_ids,
    std::vector<std::vector<KeypointType>>* p_keypoint_vectors,
    std::vector<Eigen::Vector3d>* p_objects) const {
    assert(p_object_ids != nullptr);
    assert(p_keypoint_vectors != nullptr);
    assert(p_objects != nullptr);

    // Convenience variables
    std::vector<ObjectId>& object_ids = *p_object_ids;
    std::vector<std::vector<KeypointType>>& keypoint_vectors = *p_keypoint_vectors;
    std::vector<Eigen::Vector3d>& objects = *p_objects;

    std::size_t const no_objects = object_ids_.size();

    // Reserve maximum space required in the output containers
    object_ids.clear();
    object_ids.reserve(no_objects);
    keypoint_vectors.clear();
    keypoint_vectors.reserve(no_objects);
    objects.clear();
    objects.reserve(no_objects);

    // Actually properly fill the output containers
    for (ObjectId const& object_id : object_ids_) {
        std::vector<std::pair<FrameId, KeypointType>> const& keypoints = all_keypoints_[object_id];

        std::vector<KeypointType> result(size, default_keypoint_);

        // Get the required number keypoints of the last frames
        std::size_t const copy_size = std::min(keypoints.size(), size);

        bool valid = false;

        std::size_t const offset = size - copy_size;
        for (std::size_t j = 0; j < copy_size; ++j) {
            Keypoint const& kp = keypoints[keypoints.size() - copy_size + j].second;
            // Stop when no keypoints are available any more
            if (kp == default_keypoint_) {
                break;
            }
            result[offset + j] = kp;
            // When there are 2 or more keypoints, then this object can be added.
            if (j == 1) {
                valid = true;
            }
        }

        if (valid) {
            object_ids.push_back(object_id);
            keypoint_vectors.push_back(std::move(result));
            objects.push_back(all_objects_[object_id]);
        }
    }
}

void VisualOdometryObjectKeypoints::Pop(
    ObjectId const& object_id, Eigen::Vector3d* p_object, std::vector<std::pair<FrameId, KeypointType>>* p_keypoints) {
    CheckObjectId(object_id);

    if (p_object != nullptr) {
        *p_object = std::move(all_objects_[object_id]);
    }
    if (p_keypoints != nullptr) {
        std::vector<std::pair<FrameId, KeypointType>>& keypoints = *p_keypoints;
        keypoints = std::move(all_keypoints_[object_id]);
        keypoints.erase(
            std::remove_if(
                keypoints.begin(), keypoints.end(), [this](auto const& kp) { return kp.second == default_keypoint_; }),
            keypoints.end());
    } else {
        all_keypoints_[object_id].clear();
    }
    object_ids_.erase(object_id);
}

VisualOdometryObjectKeypoints::ObjectId VisualOdometryObjectKeypoints::AddNewObject() {
    all_keypoints_.emplace_back();
    all_objects_.emplace_back();
    object_ids_.insert(all_objects_.size() - 1);
    return all_objects_.size() - 1;
}

void VisualOdometryObjectKeypoints::CheckObjectId(ObjectId const& object_id) const {
    if (!ValidObjectId(object_id)) {
        LOG(FATAL) << "Object id requested is not available any more.";
    }
}

}  // namespace detail

}  // namespace nie
