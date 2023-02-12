/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "transform.hpp"

namespace nie {

cv::Mat GetLookUpTable(
    std::function<bool(cv::Point2f const& p_u, cv::Point2f* p_d)> const& transformer, cv::Size const& size) {
    // Lookup table for remapping
    cv::Mat lut(size.height, size.width, CV_32FC2);

    // TODO(jbr) parallel for ? means transformer should not create dependencies
    // Fill lookup table.
    for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
            // From pixel index to the center of the pixel. The math assumes the center of the most top left pixel to be
            // 0.5, 0.5.
            cv::Point2f p_b(static_cast<float>(x), static_cast<float>(y));

            cv::Point2f p_a;

            // Only do the actual remapping in case the resulting coordinates fall within the image bounds.
            // The opencv remap function already skips pixels that fall outside of the image bounds, so this part mostly
            // skips back projections to counter mirroring effects, etc.
            if (transformer(p_b, &p_a)) {
                lut.at<cv::Point2f>(y, x) = p_a;
            } else {
                // These get skipped in cv::remap() and most other bounds checking remappers (need to set "something").
                lut.at<cv::Point2f>(y, x) = cv::Point2f(-2.0f, -2.0f);
            }
        }
    }

    return lut;
}

}  // namespace nie