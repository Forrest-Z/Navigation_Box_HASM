/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "pose_bbox.hpp"

namespace nie {

bool CheckIntersectionArea(
        nie::PoseBbox const& bounds_a,
        nie::PoseBbox const& bounds_b,
        double intersection_area_threshold,
        double intersection_iou_threshold);

}  // namespace nie
