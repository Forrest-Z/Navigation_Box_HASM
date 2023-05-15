/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "descriptor_extractor_lv_edge.hpp"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace detail {

std::vector<cv::Point2f> const kPois{
        {-1, -5},
        {+1, -5},
        {-1, +5},
        {+1, +5},
        {-5, -3},
        {+5, -3},
        {-5, +3},
        {+5, +3},
        {-3, -1},
        {+3, -1},
        {-3, +1},
        {+3, +1},
        {-1, -1},
        {+1, -1},
        {-1, +1},
        {+1, +1}};

}  // namespace detail

namespace nie {

DescriptorVector DescriptorExtractorLvEdge::Describe(cv::Mat const& image, KeypointVector const& features) {
    cv::Mat filter_x, filter_y;
    Filter(image, &filter_x, &filter_y);

    Callback<Handle::kFilterX>(filter_x, features);
    Callback<Handle::kFilterY>(filter_y, features);

    return GetDescriptors(filter_x, filter_y, features);
}

void DescriptorExtractorLvEdge::Filter(cv::Mat const& image, cv::Mat* p_filter_x, cv::Mat* p_filter_y) {
    // By default the Sobel function has border type BORDER_DEFAULT, which is equal to BORDER_REFLECT_101, meaning that
    // image pixel at position -1 will be taken to be the same as at location 1 etc. Therefore the Sobel response of a
    // pixel at the left border (x=0) will have response 0.
    assert(p_filter_x != nullptr);
    assert(p_filter_y != nullptr);
    cv::Sobel(
            image,
            *p_filter_x,
            kFeatureDescriptorFilterDepth,
            /* dx */ 1,
            /* dy */ 0);
    cv::Sobel(
            image,
            *p_filter_y,
            kFeatureDescriptorFilterDepth,
            /* dx */ 0,
            /* dy */ 1);
}

DescriptorVector DescriptorExtractorLvEdge::GetDescriptors(
        cv::Mat const& filter_x, cv::Mat const& filter_y, KeypointVector const& features) {
    DescriptorVector result(features.size(), FeatureDescriptor{});

    for (std::size_t i = 0; i < features.size(); ++i) {
        Keypoint const& feature = features[i];
        FeatureDescriptor& descriptor = result[i];
        FillDescriptor(filter_x, feature, &descriptor);
        FillDescriptor(filter_y, feature, &descriptor, false);
    }

    return result;
}

void DescriptorExtractorLvEdge::FillDescriptor(
        cv::Mat const& filter, Keypoint const& feature, FeatureDescriptor* p_descriptor, bool first_half) {
    assert(p_descriptor != nullptr);

    std::size_t offset = (first_half ? 0 : ::detail::kPois.size());

    auto& descriptor = *p_descriptor;
    cv::Rect const bounding_box(cv::Point{}, filter.size());
    for (std::size_t index = 0; index < ::detail::kPois.size(); ++index) {
        Keypoint const& p = feature + ::detail::kPois[index];
        descriptor[index + offset] = bounding_box.contains(p) ? filter.at<FeatureDescriptorType>(p) : 0;
    }
}

}  // namespace nie
