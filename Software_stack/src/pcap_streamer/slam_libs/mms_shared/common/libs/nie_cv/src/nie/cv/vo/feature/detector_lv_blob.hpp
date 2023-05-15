/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_DETECTOR_LV_BLOB_HPP
#define NIE_CV_VO_FEATURE_DETECTOR_LV_BLOB_HPP

#include "detector.hpp"

#include <nie/core/callbacks.hpp>

namespace nie {

namespace detail {

enum class DetectorLvBlobHandle : std::size_t { kFilter };

}  // namespace detail

/*
 * LibViso2-based blob detector
 * Reference: "SteroScan: Dense 3d Reconstruction in Real-time" by Geiger,
 *            Ziegler and Stiller, 2011
 */
class DetectorLvBlob final
    : public Detector,
      public Callbacks<detail::DetectorLvBlobHandle, std::function<void(cv::Mat const&, KeypointVector const&)>> {
public:
    explicit DetectorLvBlob(Parameters parameters = Parameters())
        : Detector(parameters, KeypointType::kBlobMax, KeypointType::kBlobMin) {}
    ~DetectorLvBlob() override = default;

    void Detect(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) override;

private:
    cv::Mat Filter(cv::Mat const& image);
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_DETECTOR_LV_BLOB_HPP
