/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "detector.hpp"

#include <nie/core/scoped_timer.hpp>
#include "nie/cv/non_maximum_suppression.hpp"

namespace nie {

int Detector::IncreaseDepth(int depth) {
    // Initialise with fall back value
    int result = CV_64F;

    switch (depth) {
        case CV_8U:
        case CV_8S: {
            result = CV_16S;
            break;
        }
        case CV_16U:
        case CV_16S: {
            result = CV_32F;
            break;
        }
        default: {}
    }
    return result;
}

void Detector::FindFeatures(cv::Mat const& image, KeypointVector* p_features, KeypointTypeVector* p_feature_types) {
    ScopedTimer timer("Detector::FindFeatures");
    assert(p_features != nullptr);
    assert(p_feature_types != nullptr);

    // Get the extreme values from the filter response
    std::vector<bool> extremes;
    FindLocalExtremes(image, parameters_.feature_distance, p_features, &extremes);

    if (extremes.empty()) {
        return;
    }

    // Extend KeypointTypeVector
    // detail: method below best compared to emplace_back, conditional update, std::transform or std::generate
    std::size_t offset = p_feature_types->size();
    p_feature_types->resize(offset + extremes.size(), max_type_);
    for (std::size_t i = 0; i < extremes.size(); ++i) {
        (*p_feature_types)[offset + i] = (extremes[i] ? max_type_ : min_type_);
    }
}

}  // namespace nie
