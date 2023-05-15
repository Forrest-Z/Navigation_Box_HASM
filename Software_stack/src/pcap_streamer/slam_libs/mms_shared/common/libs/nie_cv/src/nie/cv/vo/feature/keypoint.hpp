/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_FEATURE_KEYPOINT_HPP
#define NIE_CV_FEATURE_KEYPOINT_HPP

#include <vector>

#include <opencv2/opencv.hpp>

namespace nie {

// Keypoint type is chosen to be an OpenCV type, because they are mostly used by OpenCV functions. Another reason is
// that they can be mapped to Eigen types, such that Eigen functions can be used (not the other way around).
// TODO(jbr): Investigate reduction of KeypointVector copies.
using Keypoint = cv::Point2f;
using KeypointVector = std::vector<Keypoint>;
enum class KeypointType { kUnknown, kBlobMax, kBlobMin, kCornerMax, kCornerMin };
using KeypointTypeVector = std::vector<KeypointType>;

}  // namespace nie

#endif  // NIE_CV_FEATURE_KEYPOINT_HPP
