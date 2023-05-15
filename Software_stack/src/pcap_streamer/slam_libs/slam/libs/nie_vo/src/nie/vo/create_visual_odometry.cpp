/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "create_visual_odometry.hpp"

#include <nie/cv/vo/feature/descriptor_extractor_lv_edge.hpp>
#include <nie/cv/vo/feature/detector_lv_blob.hpp>
#include <nie/cv/vo/feature/detector_lv_corner.hpp>
#include <nie/cv/vo/feature/match_filter_bucketing.hpp>
#include <nie/cv/vo/feature/match_filter_local_flow.hpp>
#include <nie/cv/vo/feature/matcher_lv.hpp>

#include "draw.hpp"

namespace nie {

namespace detail {

constexpr float kScale = 0.45;

std::vector<DetectorPtr> CreateDetectors(bool draw) {
    std::vector<DetectorPtr> detectors(2);
    DetectorLvBlob detector_blob;
    if (draw) {
        detector_blob.AddCallback<DetectorLvBlob::Handle::kFilter>(
            [](cv::Mat const& filter, KeypointVector const& features) {
                DrawImage("Blob filter", MakeImageFeatures(ConvertFilter(filter), features), kScale);
            });
    }
    detectors[0] = std::make_unique<DetectorLvBlob>(detector_blob);
    DetectorLvCorner detector_corner;
    if (draw) {
        detector_corner.AddCallback<DetectorLvCorner::Handle::kFilter>(
            [](cv::Mat const& filter, KeypointVector const& features) {
                DrawImage("Corner filter", MakeImageFeatures(ConvertFilter(filter), features), kScale);
            });
    }
    detectors[1] = std::make_unique<DetectorLvCorner>(detector_corner);
    return detectors;
}

DescriptorExtractorPtr CreateDescriptorExtractor(bool draw) {
    DescriptorExtractorLvEdge extractor_edge;
    if (draw) {
        extractor_edge.AddCallback<DescriptorExtractorLvEdge::Handle::kFilterX>(
            [](cv::Mat const& filter, KeypointVector const& features) {
                DrawImage("Sobel filter in x", MakeImageFeatures(ConvertFilter(filter), features), kScale);
            });
        extractor_edge.AddCallback<DescriptorExtractorLvEdge::Handle::kFilterY>(
            [](cv::Mat const& filter, KeypointVector const& features) {
                DrawImage("Sobel filter in y", MakeImageFeatures(ConvertFilter(filter), features), kScale);
            });
    }
    return std::make_unique<DescriptorExtractorLvEdge>(extractor_edge);
}

MatcherPtr CreateMatcher(int image_width, int image_height) {
    return std::make_unique<MatcherLv>(image_width, image_height);
}

std::vector<MatchFilterPtr> CreateMatchFilters(int image_width, int image_height) {
    std::vector<MatchFilterPtr> filters(2);
    filters[0] = std::make_unique<MatchFilterLocalFlow>(image_width, image_height);
    filters[1] = std::make_unique<MatchFilterBucketing>(image_width, image_height);
    return filters;
}

}  // namespace detail

VisualOdometryMonoPtr CreateVisualOdometryMono(
    int image_width, int image_height, VisualOdometryMono::Parameters parameters, bool draw) {
    VisualOdometryConfiguration conf{detail::CreateDetectors(draw),
                                     detail::CreateDescriptorExtractor(draw),
                                     detail::CreateMatcher(image_width, image_height),
                                     detail::CreateMatchFilters(image_width, image_height)};
    return std::make_unique<VisualOdometryMono>(image_width, image_height, std::move(conf), std::move(parameters));
}

}  // namespace nie
