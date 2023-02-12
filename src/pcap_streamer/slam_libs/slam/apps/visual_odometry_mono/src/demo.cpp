/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "demo.hpp"

#include <functional>

#include <nie/vo/draw.hpp>
#include <opencv2/opencv.hpp>

namespace detail {

void DrawFeatures(
    nie::FrameId const& image_id,
    std::vector<std::string> const& image_files,
    nie::KeypointVector features,
    float scale,
    int image_type) {
    static cv::Mat prev_image;
    static nie::KeypointVector prev_features;

    cv::Mat image = cv::imread(image_files[image_id], image_type);

    cv::Mat result;
    if (prev_image.empty()) {
        result = nie::MakeImageFeatures(image, features);
    } else {
        result = nie::MakeImageFeatures(prev_image, prev_features, image, features);
    }
    nie::DrawImage("Features found", result, scale);

    cv::swap(prev_image, image);
    std::swap(prev_features, features);
}

void DrawMatches(
    nie::FrameId const& image_id,
    std::vector<std::string> const& image_files,
    nie::KeypointVector const& features_a,
    nie::KeypointVector const& features_b,
    nie::MatchVector const& matches,
    nie::MatchVector const& matches_filtered,
    float scale,
    int image_type) {
    if (!features_a.empty()) {
        cv::Mat image = cv::imread(image_files[image_id], image_type);
        cv::Mat result = nie::MakeImageMatches(image, features_a, features_b, matches, matches_filtered);
        nie::DrawImage("Feature matches in last 2 key frames (colored after motion)", result, scale);
    }
}

void DrawPose(nie::FrameId, nie::Isometry3md const& pose) {
    static nie::Trajectory trajectory;

    trajectory.Update(pose.translation()[0], pose.translation()[2]);

    // Actually draw the trajectory
    nie::DrawImage("Trajectory", trajectory.MakeImage());
}

void DrawBackProjections(
    std::string const& name,
    std::vector<nie::FrameId> const& image_ids,
    std::vector<std::string> const& image_files,
    std::vector<nie::KeypointVector> const& feature_vectors,
    std::vector<nie::KeypointVector> const& back_projected_feature_vectors,
    float scale,
    int image_type) {
    assert(image_ids.size() == feature_vectors.size());
    assert(feature_vectors.size() == back_projected_feature_vectors.size());

    cv::Mat result;
    for (std::size_t i = 0; i < image_ids.size(); ++i) {
        cv::Mat image = cv::imread(image_files[image_ids[i]], image_type);
        cv::Mat annotated_image =
            nie::MakeImageBackProjection(image, feature_vectors[i], back_projected_feature_vectors[i]);
        if (result.empty()) {
            cv::swap(annotated_image, result);
        } else {
            cv::hconcat(result, annotated_image, result);
        }
    }
    nie::DrawImage(name, result, scale);
}

}  // namespace detail

void AddDrawingCallbacks(
    std::vector<std::string> const& image_files, float scale, int image_type, nie::VisualOdometryMonoPtr* p_vo) {
    nie::VisualOdometryMonoPtr& vo = *p_vo;

    // clang-format off
    using namespace std::placeholders;
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawFeatures>(std::bind(
        detail::DrawFeatures, _1, image_files, _2, scale, image_type));
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawMatches>(std::bind(
        detail::DrawMatches, _1, image_files, _2, _3, _4, _5, scale, image_type));
    vo->AddCallback<nie::VisualOdometryMono::Handle::kWritePose>(
        detail::DrawPose);
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawBackProjectionsTriangulation>(std::bind(
        detail::DrawBackProjections, "Back-projections of triangulations in last 2 key frames",
        _1, image_files, _2, _3, scale, image_type));
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawBackProjectionsBeforeBundleAdjustment>(std::bind(
        detail::DrawBackProjections, "Back-projections of before bundle adjustment",
        _1, image_files, _2, _3, scale, image_type));
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawBackProjectionsBetweenBundleAdjustment>(std::bind(
        detail::DrawBackProjections, "Back-projections of between bundle adjustment",
        _1, image_files, _2, _3, scale, image_type));
    vo->AddCallback<nie::VisualOdometryMono::Handle::kDrawBackProjectionsAfterBundleAdjustment>(std::bind(
        detail::DrawBackProjections, "Back-projections of after bundle adjustment",
        _1, image_files, _2, _3, scale, image_type));
    // clang-format on
}
