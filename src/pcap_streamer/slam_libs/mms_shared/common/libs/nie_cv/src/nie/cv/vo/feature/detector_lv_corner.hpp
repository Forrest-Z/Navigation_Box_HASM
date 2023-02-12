/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_DETECTOR_LV_CORNER_HPP
#define NIE_CV_VO_FEATURE_DETECTOR_LV_CORNER_HPP

#include "detector.hpp"

#include <nie/core/callbacks.hpp>

namespace nie {

namespace detail {

enum class DetectorLvCornerHandle : std::size_t { kFilter };

}  // namespace detail

/*
 * LibViso2-based corner detector
 * Reference: "SteroScan: Dense 3d Reconstruction in Real-time" by Geiger,
 *            Ziegler and Stiller, 2011
 */
class DetectorLvCorner final
    : public Detector,
      public Callbacks<detail::DetectorLvCornerHandle, std::function<void(cv::Mat const&, KeypointVector const&)>> {
public:
    explicit DetectorLvCorner(Parameters parameters = Parameters())
        : Detector(parameters, KeypointType::kCornerMax, KeypointType::kCornerMin) {}
    ~DetectorLvCorner() override = default;

    void Detect(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) override;

private:
    cv::Mat Filter(cv::Mat const& image);
};

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_DETECTOR_LV_CORNER_HPP
