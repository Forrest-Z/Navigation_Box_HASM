/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_DETECTOR_HPP
#define NIE_CV_VO_FEATURE_DETECTOR_HPP

#include "keypoint.hpp"

namespace nie {

/// Abstract class specifying the interface for feature detectors
class Detector {
public:
    struct Parameters {
        Parameters()
            : feature_distance(3)  // 3
        {}

        int feature_distance;
    };

    virtual ~Detector() = default;

    /// Function that will add the detected keypoints to the position and type vectors.
    virtual void Detect(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) = 0;

protected:
    explicit Detector(
        Parameters const parameters,
        KeypointType const max_type = KeypointType::kUnknown,
        KeypointType const min_type = KeypointType::kUnknown)
        : parameters_(parameters), max_type_(max_type), min_type_(min_type) {}

    static int IncreaseDepth(int depth);

    void FindFeatures(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types);

    Parameters parameters_;

    KeypointType max_type_;
    KeypointType min_type_;
};
using DetectorPtr = std::unique_ptr<Detector>;

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_DETECTOR_HPP
