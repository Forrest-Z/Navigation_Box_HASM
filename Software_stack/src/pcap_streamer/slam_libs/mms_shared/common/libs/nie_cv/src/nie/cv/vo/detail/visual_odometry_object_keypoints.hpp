/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_OBJECT_KEYPOINTS_HPP
#define NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_OBJECT_KEYPOINTS_HPP

#include <unordered_set>

#include <Eigen/Dense>
#include <nie/core/hash.hpp>

#include "nie/cv/vo/feature/keypoint.hpp"
#include "nie/cv/vo/types.hpp"

namespace nie {

namespace detail {

class VisualOdometryObjectKeypoints {
private:
    using ObjectId = std::size_t;
    using KeypointType = Keypoint;

public:
    explicit VisualOdometryObjectKeypoints(KeypointType const& kp)
        : map_object_id_(), object_ids_(), all_keypoints_(), all_objects_(), default_keypoint_(kp) {}

    std::pair<ObjectId, bool> GetObjectId(FrameId const& id, std::size_t const& match_index);
    void SetObjectId(ObjectId const& object_id, FrameId const& id, std::size_t const& match_index);

    // To be used by user if object id is still valid to used or that match chain / object is already discarded
    bool ValidObjectId(ObjectId const& object_id) const;

    void PrepareKeypoints(FrameId const& id);
    void AddKeypoint(ObjectId const& object_id, FrameId const& id, KeypointType const& keypoint);
    void SetKeypoint(ObjectId const& object_id, FrameId const& id, KeypointType const& keypoint);
    void SetObject(ObjectId const& object_id, Eigen::Vector3d object);

    Eigen::Vector3d const& GetObject(ObjectId const& object_id) { return all_objects_[object_id]; }

    std::vector<ObjectId> GetObjectIdsToWrite(std::size_t const overlap) const;

    // Will return data in the format as used by the bundle adjustment
    void GetBaData(
        std::size_t const& size,
        std::vector<ObjectId>* p_object_ids,
        std::vector<std::vector<KeypointType>>* p_keypoint_vectors,
        std::vector<Eigen::Vector3d>* p_objects) const;

    void Pop(
        ObjectId const& object_id,
        Eigen::Vector3d* p_object = nullptr,
        std::vector<std::pair<FrameId, KeypointType>>* p_keypoints = nullptr);

    void debug(bool stop = false) const;

private:
    ObjectId AddNewObject();

    void CheckObjectId(ObjectId const& object_id) const;

    // Custom types
    using Key = std::pair<FrameId, std::size_t>;  // {keyframe id, match vector index}
    using Map = std::unordered_map<Key, ObjectId, nie::PairHash>;
    using Iter = Map::iterator;

    Map map_object_id_;  // {keyframe id, match vector index} -> object id
    std::unordered_set<ObjectId> object_ids_;
    std::vector<std::vector<std::pair<FrameId, KeypointType>>> all_keypoints_;
    std::vector<Eigen::Vector3d> all_objects_;

    KeypointType const default_keypoint_;

    // TODO(MvB): The data structure can be simplified and the frame ids can be deduplicated, using something like:
    // Map map_object_id_;  // {keyframe id, match vector index} -> object id
    // std::vector<FrameId> object_ids_;
    // struct ObjectKeypoints {
    //    std::size_t frame_index;
    //    Eigen::Vector3d object;
    //    std::vector<KeypointType> keypoints;
    // };
    // std::unordered_map<ObjectId, ObjectKeypoints> object_keypoints_;
};

}  // namespace detail

}  // namespace nie

#endif  // NIE_CV_VO_DETAIL_VISUAL_ODOMETRY_OBJECT_KEYPOINTS_HPP
