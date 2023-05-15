/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_DESCRIPTOR_EXTRACTOR_LV_EDGE_HPP
#define NIE_CV_VO_FEATURE_DESCRIPTOR_EXTRACTOR_LV_EDGE_HPP

#include "descriptor_extractor.hpp"

#include <nie/core/callbacks.hpp>

namespace nie {

namespace detail {

enum class DescriptorExtractorLvEdgeHandle : std::size_t { kFilterX, kFilterY };

}  // namespace detail

/*
 * LibViso2-based feature descriptor extractor
 * Reference: "SteroScan: Dense 3d Reconstruction in Real-time" by Geiger,
 *            Ziegler and Stiller, 2011
 */
class DescriptorExtractorLvEdge : public DescriptorExtractor,
                                  public Callbacks<
                                      detail::DescriptorExtractorLvEdgeHandle,
                                      std::function<void(cv::Mat const&, KeypointVector const&)>,
                                      std::function<void(cv::Mat const&, KeypointVector const&)>> {
public:
    ~DescriptorExtractorLvEdge() override = default;

    DescriptorVector Describe(cv::Mat const& image, KeypointVector const& features) override;

private:
    static void Filter(cv::Mat const& image, cv::Mat* p_filter_x, cv::Mat* p_filter_y);

    static DescriptorVector GetDescriptors(
        cv::Mat const& filter_x, cv::Mat const& filter_y, KeypointVector const& features);

    static void FillDescriptor(
        cv::Mat const& filter, Keypoint const& feature, FeatureDescriptor* p_descriptor, bool first_half = true);
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_DESCRIPTOR_EXTRACTOR_LV_EDGE_HPP
