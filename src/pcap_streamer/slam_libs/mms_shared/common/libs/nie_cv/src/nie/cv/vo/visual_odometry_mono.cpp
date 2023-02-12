/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "visual_odometry_mono.hpp"

#include <glog/logging.h>

#include <nie/core/algorithm.hpp>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/scoped_timer.hpp>

#include "ba_problem.hpp"
#include "back_projection.hpp"
#include "visual_odometry.hpp"

namespace nie {

VisualOdometryMono::VisualOdometryMono(
        std::size_t const image_width,
        std::size_t const image_height,
        VisualOdometryConfiguration configuration,
        Parameters parameters)
    : image_width_(image_width),
      image_height_(image_height),
      configuration_(std::move(configuration)),
      parameters_(std::move(parameters)),
      cv_K_(nie::ConvertMat(parameters_.K)),
      state_(parameters_),
      // Taking N to be the number of key frames to be considered for bundle adjustment, the size of different
      // containers required are
      //   * TrackingInfo: only one instance needed in order to match the current with the previous features
      //   * FrameId: N instances needed to know the id of all key frames
      //   * KeypointVector: N instances needed to know the feature positions of all key frames
      //   * Isometry3qd: N instances needed to know the calculated positions of all key frames
      //   * MatchVector: N - 1 instances needed to store the matches between the key frames
      //   * 3d objects: N - 1 instances needed to store the objects based on the matches between the key frames
      prev_frame_(),
      scale_factor_(1.),
      prev_ids_(parameters_.ba_window_size),
      prev_features_(parameters_.ba_window_size),
      prev_poses_(parameters_.ba_window_size),
      prev_matches_(parameters_.ba_window_size - 1),
      prev_objects_(parameters_.ba_window_size - 1),
      object_keypoints_(kDefaultKeypoint) {
    // Sanity checks
    CHECK(parameters_.keyframe_window_size > 1) << "Keyframe window size should be larger than 1.";
    CHECK(parameters_.ba_window_size > 1) << "Bundle adjustment window size should be larger than 1.";
    CHECK(parameters_.ba_window_overlap < parameters_.ba_window_size)
            << "Bundle adjustment overlap should be smaller its window size.";
}

bool VisualOdometryMono::Process(FrameId const& id, cv::Mat const& image) {
    nie::ScopedTimer timer("VisualOdometryMono::Process");
    assert(!image.empty());

    bool result = false;

    TrackingInfo current_frame = DetectAndMatchFeatures(id, image);

    if (state_.IsFirstImage()) {
        prev_ids_.PushBack(id);
        prev_features_.PushBack(current_frame.features);
        prev_poses_.PushBack(nie::Isometry3md::Identity());
        current_frame.matches.clear();
    } else if (state_.IsKeyFrame()) {
        // FIXME(MvB): Multiple functions use FindLinkedMatches on the same two match vectors, this should be avoided.
        // Also to be investigated if this is also happening in other cases.

        Isometry3md motion;
        result = DetermineMotion(&current_frame, &motion);

        if (result) {
            // As the motion detection is OK, this frame will actually be stored as a key frame
            prev_ids_.PushBack(id);
            prev_features_.PushBack(current_frame.features);
            prev_matches_.PushBack(current_frame.matches);
            // Reset the matches for the frames in the next window
            current_frame.matches.clear();

            // Apply the same scale factor to the current motion as done for the last motion
            motion.translation() *= scale_factor_;
            std::vector<Eigen::Vector3d> objects = DetermineObjects(motion);

            prev_objects_.PushBack(std::move(objects));

            if (state_.IsFirstMotion()) {
                PrepareBaObjects();
            } else {
                double scale_factor = DetermineScale();
                if (scale_factor != 1.) {
                    motion.translation() *= scale_factor;
                    scale_factor_ *= scale_factor;

                    // Updated motion, so update the 3D objects
                    objects = DetermineObjects(motion);
                    std::swap(prev_objects_.Back(), objects);
                }

                AppendBaObjects();
            }
            prev_poses_.PushBack(prev_poses_.Back() * motion);

            if (state_.IsBaFrame()) {
                PerformBundleAdjustment(parameters_.back_projection_error);
                PublishResults();
            }
        } else {
            LOG(ERROR) << "No motion is determined, skipping frame.";
            // No state update, so ignoring this frame as keyframe and also ignore features tracking (try to use next
            // frame as key frame)
            return result;
        }
    }

    std::swap(prev_frame_, current_frame);
    state_.Next();

    return result;
}

VisualOdometryMono::TrackingInfo VisualOdometryMono::DetectAndMatchFeatures(
        FrameId const& id, cv::Mat const& image) const {
    nie::ScopedTimer timer("VisualOdometryMono::DetectAndMatchFeatures");

    // Detect features in the new image
    TrackingInfo current_frame;
    current_frame.id = id;
    configuration_.DetectFeatures(image, &current_frame.features, &current_frame.feature_types);
    current_frame.descriptors = configuration_.DescribeFeatures(image, current_frame.features);

    VLOG(2) << "Features detected: " << current_frame.features.size();

    Callback<Handle::kDrawFeatures>(id, current_frame.features);

    // Match the new features with the ones from the previous image
    if (!state_.IsFirstImage()) {
        // Find the matches between the features from previous image and the newly found ones
        current_frame.matches = configuration_.MatchFeatures(
                prev_frame_.features,
                prev_frame_.feature_types,
                prev_frame_.descriptors,
                current_frame.features,
                current_frame.feature_types,
                current_frame.descriptors,
                prev_frame_.matches);

        // Sorting the matches, as this is required for the linking of matches at later stages
        std::sort(current_frame.matches.begin(), current_frame.matches.end());

        // Combine the current matches with the previous ones
        if (!prev_frame_.matches.empty()) {
            current_frame.matches = ConcatenateMatches(prev_frame_.matches, current_frame.matches);
            VLOG(2) << "Tracked matches: " << current_frame.matches.size();
        } else {
            VLOG(2) << "New matches: " << current_frame.matches.size();
        }
    }

    return current_frame;
}

bool VisualOdometryMono::DetermineMotion(TrackingInfo* p_current_frame, Isometry3md* p_motion) const {
    nie::ScopedTimer timer("VisualOdometryMono::DetermineMotion");
    assert(p_current_frame != nullptr);
    assert(p_motion != nullptr);

    // Convenience variables
    KeypointVector const& prev_features = prev_features_.Back();
    KeypointVector const& features = p_current_frame->features;

    // Create copy of matches for
    //   - filtering matches and only updating the input when every succeeded, and
    //   - the callback "kDrawMatches" that visualizes all/original and filtered/new matches
    MatchVector new_matches = p_current_frame->matches;

    // Filter the matches using different outlier removal strategies (does not preserve order)
    configuration_.FilterMatches(prev_features, features, &new_matches);
    VLOG(2) << "Matches after simple filtering: " << new_matches.size();

    Isometry3md new_motion = Isometry3md::Identity();

    bool valid = ValidateGlobalFlow(
            prev_features, features, new_matches, image_width_, image_height_, parameters_.global_flow_threshold);
    if (valid) {
        // Estimate the motion based on the matches and only keep the matches that comply with the motion (does not
        // preserve order)
        valid = EstimateMotion(prev_features, features, cv_K_, &new_matches, &new_motion);

        if (valid) {
            VLOG(2) << "Matches after motion filtering: " << new_matches.size();
        } else {
            VLOG(2) << "Motion estimation failed.";
        }
    } else {
        VLOG(2) << "Matches not valid for motion estimation.";
    }

    Callback<Handle::kDrawMatches>(p_current_frame->id, prev_features, features, p_current_frame->matches, new_matches);

    // When everything succeeded, then update the inputs.
    if (valid) {
        std::swap(*p_motion, new_motion);

        // Ensure sorted matches again for later linking of matches
        std::sort(new_matches.begin(), new_matches.end());
        std::swap(p_current_frame->matches, new_matches);
    }

    return valid;
}

std::vector<Eigen::Vector3d> VisualOdometryMono::DetermineObjects(Isometry3md const& motion) const {
    nie::ScopedTimer timer("VisualOdometryMono::DetermineObjects");

    // Convenience variables
    KeypointVector const& prev_features = prev_features_[-2];
    KeypointVector const& features = prev_features_[-1];
    MatchVector const& matches = prev_matches_.Back();

    Isometry3md const& prev_pose = prev_poses_[-1];
    Isometry3md const pose = prev_pose * motion;

    // "Misuse" ChainMatches function to split the matches as a vector of vectors
    std::vector<std::vector<std::size_t>> matches_bc_feature_indices = ChainMatches({matches});

    std::vector<Eigen::Vector3d> objects;
    nie::TriangulateFeatures(
            std::vector<Isometry3md>{prev_pose, pose},
            {prev_features, features},
            matches_bc_feature_indices,
            parameters_.K,
            &objects);

    if (HasCallback<Handle::kDrawBackProjectionsTriangulation>()) {
        std::vector<KeypointVector> orig_kpnts(2);
        orig_kpnts.front().reserve(matches.size());
        orig_kpnts.back().reserve(matches.size());
        for (FeatureMatch const& m : matches) {
            orig_kpnts.front().push_back(prev_features[m.index_a]);
            orig_kpnts.back().push_back(features[m.index_b]);
        }

        std::vector<KeypointVector> bp_kpnts(2);
        bp_kpnts.front().reserve(matches.size());
        bp_kpnts.back().reserve(matches.size());
        CalculateBackProjection(objects, prev_pose, parameters_.K, &bp_kpnts.front());
        CalculateBackProjection(objects, pose, parameters_.K, &bp_kpnts.back());

        Callback<Handle::kDrawBackProjectionsTriangulation>(
                std::vector<FrameId>{prev_ids_[-2], prev_ids_[-1]}, orig_kpnts, bp_kpnts);
    }

    return objects;
}

double VisualOdometryMono::DetermineScale() const {
    return ComputeRelativeFactor(prev_matches_[-2], prev_matches_[-1], prev_objects_[-2], prev_objects_[-1]);
}

void VisualOdometryMono::PrepareBaObjects() {
    nie::ScopedTimer timer("VisualOdometryMono::PrepareBaObjects");

    FrameId const& prev_id = prev_ids_[0];
    FrameId const& id = prev_ids_[1];
    KeypointVector const& prev_features = prev_features_[0];
    KeypointVector const& features = prev_features_[1];

    MatchVector const& matches = prev_matches_.Front();
    std::vector<Eigen::Vector3d> const& objects = prev_objects_.Front();

    // Prepare for the new keypoints
    for (std::size_t i = 0; i < matches.size(); ++i) {
        FeatureMatch const& match = matches[i];
        std::pair<std::size_t, bool> result = object_keypoints_.GetObjectId(id, i);
        assert(result.second);  // All newly inserted / true
        std::size_t const& object_id = result.first;
        object_keypoints_.AddKeypoint(object_id, prev_id, prev_features[match.index_a]);
        object_keypoints_.AddKeypoint(object_id, id, features[match.index_b]);
        object_keypoints_.SetObject(object_id, objects[i]);
    }
}

void VisualOdometryMono::AppendBaObjects() {
    nie::ScopedTimer timer("VisualOdometryMono::AppendBaObjects");

    FrameId const& prev_frame_id = prev_ids_[-2];
    FrameId const& frame_id = prev_ids_[-1];
    KeypointVector const& prev_features = prev_features_[-2];
    KeypointVector const& features = prev_features_[-1];
    MatchVector const& prev_matches = prev_matches_[-2];
    MatchVector const& matches = prev_matches_[-1];
    std::vector<Eigen::Vector3d> objects = prev_objects_[-1];

    // Find common matches (where index_a.index_b == index_b.index_a)
    std::vector<std::size_t> prev_match_indices, match_indices;
    FindLinkedMatches(prev_matches, matches, &prev_match_indices, &match_indices);

    VLOG(2) << "Common matches between key frames: " << prev_match_indices.size();

    // Prepare for the new keypoints
    object_keypoints_.PrepareKeypoints(frame_id);
    // Common matches
    for (std::size_t i = 0; i < prev_match_indices.size(); ++i) {
        std::size_t const& prev_match_index = prev_match_indices[i];
        std::size_t const& match_index = match_indices[i];

        std::pair<std::size_t, bool> result = object_keypoints_.GetObjectId(prev_frame_id, prev_match_index);
        assert(!result.second);  // Expecting no new insert
        std::size_t const& object_id = result.first;
        if (object_keypoints_.ValidObjectId(object_id)) {
            object_keypoints_.SetObjectId(object_id, frame_id, match_index);
            object_keypoints_.SetKeypoint(object_id, frame_id, features[matches[match_index].index_b]);
        }
    }

    // All matches
    for (std::size_t match_index = 0; match_index < matches.size(); ++match_index) {
        std::pair<std::size_t, bool> result = object_keypoints_.GetObjectId(frame_id, match_index);
        std::size_t const& object_id = result.first;
        // When newly added, then also add object and previous keypoint (otherwise covered in previous "common" loop)
        if (result.second) {
            FeatureMatch const& match = matches[match_index];
            object_keypoints_.SetObject(object_id, objects[match_index]);
            object_keypoints_.AddKeypoint(object_id, prev_frame_id, prev_features[match.index_a]);
            object_keypoints_.AddKeypoint(object_id, frame_id, features[match.index_b]);
        }
    }
}

void VisualOdometryMono::PerformBundleAdjustment(float const& back_projection_error) {
    nie::ScopedTimer timer("VisualOdometryMono::PerformBundleAdjustment");

    if (prev_poses_.Count() <= 2) {
        // No use to do bundle adjustment on two poses only as they will be fixed.
        return;
    }

    // Get all required bundle adjustment data
    std::vector<std::size_t> object_ids;
    std::vector<KeypointVector> keypoint_vectors;
    std::vector<Eigen::Vector3d> objects;
    object_keypoints_.GetBaData(parameters_.ba_window_size, &object_ids, &keypoint_vectors, &objects);

    prev_poses_.Reorder();
    std::vector<Isometry3md> const& poses = prev_poses_.Data();

    // Draw back projections of bundle adjustment input
    CalculateBackProjections<Handle::kDrawBackProjectionsBeforeBundleAdjustment>(poses, keypoint_vectors, objects);

    // Do bundle adjustment (first time)
    std::vector<Isometry3qd> poses_q{poses.begin(), poses.end()};
    bool result = DoBundleAdjustment(parameters_.K, keypoint_vectors, &poses_q, &objects);

    // When result is not OK, then results will not be used
    if (!result) {
        return;
    }

    // Optional second pass bundle adjustment, after outlier matches are removed
    if (back_projection_error != -1) {
        // Draw back projections of first pass bundle adjustment result
        std::vector<Isometry3md> new_poses{poses_q.begin(), poses_q.end()};
        CalculateBackProjections<Handle::kDrawBackProjectionsBetweenBundleAdjustment>(
                new_poses, keypoint_vectors, objects);

        // Remove the outlier matches before performing bundle adjustment again by filtering the input data based on
        // object back-projection error
        BackProjectionFilterBaData(back_projection_error, new_poses, &object_ids, &keypoint_vectors, &objects);

        result = DoBundleAdjustment(parameters_.K, keypoint_vectors, &poses_q, &objects);

        // When result is not OK, then results will not be used
        if (!result) {
            return;
        }
    }

    // Update the poses and objects with the bundle adjusted values
    for (std::size_t i = 0; i < poses_q.size(); ++i) {
        prev_poses_.Data()[i] = poses_q[i];
    }
    for (std::size_t i = 0; i < object_ids.size(); ++i) {
        object_keypoints_.SetObject(object_ids[i], objects[i]);
    }

    // Draw back projections of the final bundle adjustment result
    CalculateBackProjections<Handle::kDrawBackProjectionsAfterBundleAdjustment>(poses, keypoint_vectors, objects);
}

void VisualOdometryMono::BackProjectionFilterBaData(
        float const& back_projection_error,
        std::vector<Isometry3md> const& poses,
        std::vector<std::size_t>* p_object_ids,
        std::vector<KeypointVector>* p_keypoint_vectors,
        std::vector<Eigen::Vector3d>* p_objects) {
    nie::ScopedTimer timer("VisualOdometryMono::BackProjectionFilterBaData");

    std::vector<std::size_t>& object_ids = *p_object_ids;
    std::vector<KeypointVector>& keypoint_vectors = *p_keypoint_vectors;
    std::vector<Eigen::Vector3d>& objects = *p_objects;

    // Calculate inverse poses to speed up back-projection calculations
    std::vector<Isometry3md> poses_inv{poses.begin(), poses.end()};
    std::for_each(poses_inv.begin(), poses_inv.end(), [](Isometry3md& p) { p.Inverse(); });

    // Calculate back-projections of new objects and identify which are outliers
    std::vector<std::size_t> objects_to_remove;
    for (std::size_t object_index = 0; object_index < objects.size(); ++object_index) {
        KeypointVector bp_kpnts;
        CalculateBackProjectionInv(objects[object_index], poses_inv, parameters_.K, &bp_kpnts);

        KeypointVector const& keypoints = keypoint_vectors[object_index];
        for (std::size_t frame_index = 0; frame_index < poses_inv.size(); ++frame_index) {
            Keypoint const& orig_kpnt = keypoints[frame_index];
            if (orig_kpnt != kDefaultKeypoint && cv::norm(orig_kpnt - bp_kpnts[frame_index]) > back_projection_error) {
                objects_to_remove.emplace_back(object_index);
                break;
            }
        }
    }

    // Remove the outlier objects / match chains from further processing
    for (std::size_t i : objects_to_remove) {
        object_keypoints_.Pop(object_ids[i]);
    }

    // Remove the outlier objects and do bundle adjustment again
    object_ids.erase(RemoveIf(objects_to_remove.begin(), objects_to_remove.end(), &object_ids), object_ids.end());
    keypoint_vectors.erase(
            RemoveIf(objects_to_remove.begin(), objects_to_remove.end(), &keypoint_vectors), keypoint_vectors.end());
    objects.erase(RemoveIf(objects_to_remove.begin(), objects_to_remove.end(), &objects), objects.end());
    VLOG(2) << "Number of outlier objects / match chains removed vs. the ones left in double pass bundle adjustment: "
            << objects_to_remove.size() << " vs. " << objects.size();
}

void VisualOdometryMono::PublishResults() {
    nie::ScopedTimer timer("VisualOdometryMono::PublishResults");

    for (std::size_t i = 0; i < parameters_.ba_window_size - parameters_.ba_window_overlap; ++i) {
        auto const& id = prev_ids_[i];
        Callback<Handle::kWriteInfoRef>(id);
        Callback<Handle::kWritePose>(id, prev_poses_[i]);
    }

    for (std::size_t const& object_id : object_keypoints_.GetObjectIdsToWrite(parameters_.ba_window_overlap)) {
        Eigen::Vector3d object;
        std::vector<std::pair<FrameId, Keypoint>> keypoints;
        object_keypoints_.Pop(object_id, &object, &keypoints);
        Callback<Handle::kWriteObjectAndKeypoints>(object, keypoints);
    }
}

}  // namespace nie
