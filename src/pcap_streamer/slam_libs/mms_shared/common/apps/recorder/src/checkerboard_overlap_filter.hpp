/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <opencv2/core.hpp>

namespace nie {

class BaseFilter {
public:
    virtual ~BaseFilter() = default;
    virtual std::pair<bool, bool> Filter(cv::Mat const& current_frame) = 0;
};

// doing the bookkeeping internally and receiving only the current frame is
// actually pretty sleek, and makes the filter super pluggable anywhere
class CheckerboardOverlapFilter : public BaseFilter {
public:
    CheckerboardOverlapFilter(cv::Size const& pattern_size, float overlap_threshold);
    std::pair<bool, bool> Filter(cv::Mat const& current_frame) override;
    static std::unique_ptr<BaseFilter> Create(cv::Size const& pattern_size, float overlap_threshold) {
        return std::make_unique<CheckerboardOverlapFilter>(pattern_size, overlap_threshold);
    }

private:
    std::vector<cv::Point2f> last_accepted_polygon_;
    cv::Size const pattern_size_;
    float const overlap_threshold_;
};

class NoOpFilter : public BaseFilter {
public:
    [[nodiscard]] inline std::pair<bool, bool> Filter(cv::Mat const&) override { return std::make_pair(true, true); }
    static std::unique_ptr<BaseFilter> Create() { return std::make_unique<NoOpFilter>(); }
};

}  // namespace nie
