/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_VO_VISUAL_ODOMETRY_MONO_HPP
#define NIE_CV_VO_VISUAL_ODOMETRY_MONO_HPP

#include <nie/core/callbacks.hpp>
#include <nie/core/container/circular_buffer.hpp>
#include <nie/core/geometry/isometry3.hpp>

#include "detail/visual_odometry_mono_parameters.hpp"
#include "detail/visual_odometry_object_keypoints.hpp"
#include "detail/visual_odometry_state.hpp"
#include "types.hpp"
#include "visual_odometry_configuration.hpp"

namespace nie {

namespace detail {

// Mono VO callback handles
enum class VisualOdometryMonoHandle : std::size_t {
    kWriteInfoRef,
    kWritePose,
    kWriteObjectAndKeypoints,
    kDrawFeatures,
    kDrawMatches,
    kDrawBackProjectionsTriangulation,
    kDrawBackProjectionsBeforeBundleAdjustment,
    kDrawBackProjectionsBetweenBundleAdjustment,
    kDrawBackProjectionsAfterBundleAdjustment
};

}  // namespace detail

class VisualOdometryMono final
    : public nie::Callbacks<
              detail::VisualOdometryMonoHandle,
              std::function<void(FrameId const&)>,
              std::function<void(FrameId const&, Isometry3md const&)>,
              std::function<void(Eigen::Vector3d const&, std::vector<std::pair<FrameId, Keypoint>> const&)>,
              std::function<void(FrameId const&, KeypointVector const&)>,
              std::function<void(
                      FrameId const&,
                      KeypointVector const&,
                      KeypointVector const&,
                      MatchVector const&,
                      MatchVector const&)>,
              std::function<void(
                      std::vector<FrameId> const&,
                      std::vector<KeypointVector> const&,
                      std::vector<KeypointVector> const&)>,
              std::function<void(
                      std::vector<FrameId> const&,
                      std::vector<KeypointVector> const&,
                      std::vector<KeypointVector> const&)>,
              std::function<void(
                      std::vector<FrameId> const&,
                      std::vector<KeypointVector> const&,
                      std::vector<KeypointVector> const&)>,
              std::function<void(
                      std::vector<FrameId> const&,
                      std::vector<KeypointVector> const&,
                      std::vector<KeypointVector> const&)>> {
public:
    using Parameters = detail::VisualOdometryMonoParameters;

    explicit VisualOdometryMono(
            std::size_t const image_width,
            std::size_t const image_height,
            VisualOdometryConfiguration configuration,
            Parameters parameters);

    /// This single image processing function that should be used iteratively as the new image will be processed.
    /// The function will return whether the calculation succeeded.
    /// Depending on the settings (like keyframing, windowing) final results will be published via callbacks.
    bool Process(FrameId const& id, cv::Mat const& image);

private:
    // Data storage containers only used by the pipeline regarding feature tracking
    struct TrackingInfo {
        FrameId id;
        KeypointVector features;
        KeypointTypeVector feature_types;
        DescriptorVector descriptors;
        MatchVector matches;
    };

    // Determine the features in the new image and match them to the previous ones
    TrackingInfo DetectAndMatchFeatures(FrameId const& id, cv::Mat const& image) const;

    // Determine the motion between the previous and current keyframes. Returns whether the calculation succeeded.
    // Also the properties by pointer are left untouched when calculation did not succeed.
    bool DetermineMotion(TrackingInfo* p_current_frame, Isometry3md* p_motion) const;

    // Calculate the 3D objects based on the motion between the previous and current keyframes
    std::vector<Eigen::Vector3d> DetermineObjects(Isometry3md const& motion) const;

    // Determine the relative scale of the current motion compared to the previous (using the 3D objects)
    double DetermineScale() const;

    // Add first matches to the match tracking required for bundle adjustment and writing of results
    void PrepareBaObjects();
    // Every time feature matching is done, the matches are linked as input for bundle adjustment.
    void AppendBaObjects();
    // Actually perform the bundle adjustment, possibly two times with outlier object / match chain removal based on the
    // back projection error. When back_projection_error is -1 (default), then just naive single pass is performed.
    void PerformBundleAdjustment(float const& back_projection_error = -1);
    // Function that removes the outlier matches from the input data based on object back-projection error. Note that
    // this function also updates the internal object-keypoints data container.
    void BackProjectionFilterBaData(
            float const& back_projection_error,
            std::vector<Isometry3md> const& poses,
            std::vector<std::size_t>* p_object_ids,
            std::vector<KeypointVector>* p_keypoint_vectors,
            std::vector<Eigen::Vector3d>* p_objects);

    // This function takes the bundle adjustment data as input, calculates the back-projections and publishes them
    template <Handle handle>
    void CalculateBackProjections(
            std::vector<Isometry3md> const& poses,
            std::vector<KeypointVector> keypoint_vectors,
            std::vector<Eigen::Vector3d> const& objects);

    void PublishResults();

    // Settings of the visual odometry pipeline
    std::size_t const image_width_;
    std::size_t const image_height_;
    VisualOdometryConfiguration const configuration_;
    Parameters const parameters_;
    cv::Matx33d const cv_K_;

    // Object with process variables used by the pipeline
    detail::VisualOdometryState state_;

    TrackingInfo prev_frame_;

    // Data storage containers used by the pipeline regarding motion estimation
    double scale_factor_;
    CircularBuffer<FrameId> prev_ids_;
    CircularBuffer<KeypointVector> prev_features_;
    CircularBuffer<Isometry3md> prev_poses_;
    CircularBuffer<MatchVector> prev_matches_;
    CircularBuffer<std::vector<Eigen::Vector3d>> prev_objects_;

    // Data storage container to collect the keypoints per object, used to write all of them when the object is known
    detail::VisualOdometryObjectKeypoints object_keypoints_;
};
using VisualOdometryMonoPtr = std::unique_ptr<VisualOdometryMono>;

}  // namespace nie

#include "visual_odometry_mono.inl"

#endif  // NIE_CV_VO_VISUAL_ODOMETRY_MONO_HPP
