/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_FEATURE_DESCRIPTOR_EXTRACTOR_HPP
#define NIE_CV_FEATURE_DESCRIPTOR_EXTRACTOR_HPP

#include "descriptor.hpp"
#include "keypoint.hpp"

namespace nie {

/// Abstract class specifying the interface for feature descriptor extractors
class DescriptorExtractor {
public:
    virtual ~DescriptorExtractor() = default;

    virtual DescriptorVector Describe(cv::Mat const& image, KeypointVector const& features) = 0;

protected:
    DescriptorExtractor() = default;
};
using DescriptorExtractorPtr = std::unique_ptr<DescriptorExtractor>;

}  // namespace nie

#endif  // NIE_CV_FEATURE_DESCRIPTOR_EXTRACTOR_HPP
