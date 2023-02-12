/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "helper_area.hpp"

#include <glog/logging.h>

namespace nie {

/// \brief Checks the intersection of the 2D ground planes for area and overlap.
/// \param bounds_a First bounds to intersect.
/// \param bounds_b Second bounds to intersect.
/// \param intersection_area_threshold Minimum intersection area.
/// \param intersection_iou_threshold Minimum intersection over union threshold.
bool CheckIntersectionArea(
        nie::PoseBbox const& bounds_a,
        nie::PoseBbox const& bounds_b,
        double intersection_area_threshold,
        double intersection_iou_threshold) {
    double intersection_area = 0;
    double intersection_iou = 0;
    bounds_a.Intersection2D(bounds_b, &intersection_area, &intersection_iou);
    if (intersection_area < intersection_area_threshold) {
        VLOG(4) << "The intersection is " << intersection_area << " m2 which is below the threshold value "
                << intersection_area_threshold << " m2.";
        return false;
    }
    if (intersection_iou < intersection_iou_threshold) {
        VLOG(4) << "The intersection is " << intersection_iou * 100. << "% which is below the threshold value "
                << intersection_iou_threshold * 100. << "%.";
        return false;
    }
    return true;
}

}  // namespace nie
