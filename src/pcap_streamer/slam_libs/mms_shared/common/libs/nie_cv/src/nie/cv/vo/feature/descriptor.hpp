/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_FEATURE_DESCRIPTOR_HPP
#define NIE_CV_FEATURE_DESCRIPTOR_HPP

#include <vector>

#include <opencv2/opencv.hpp>

namespace nie {

// For the current LibViso2 case, the descriptor is given by the horizontal and
// vertical (2 times) Sobel response (int16_t) at 16 locations

using FeatureDescriptorType = int16_t;
constexpr int kFeatureDescriptorSize = 2 * 16;
constexpr int kFeatureDescriptorFilterDepth = CV_16S;

using FeatureDescriptor = cv::Vec<FeatureDescriptorType, kFeatureDescriptorSize>;
using DescriptorVector = std::vector<FeatureDescriptor>;

}  // namespace nie

#endif  // NIE_CV_FEATURE_DESCRIPTOR_HPP
