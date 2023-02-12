/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "color_conversion.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>

namespace nie {

pcl::PointXYZHSV MakePointXYZHSV(float h, float v) {
    pcl::PointXYZHSV p_hsv;
    p_hsv.h = h;
    p_hsv.s = 1.0f;
    p_hsv.v = v;
    return p_hsv;
}

pcl::PointXYZRGB MakePointXYZRGB(float h, float v) {
    pcl::PointXYZRGB p_rgb;
    pcl::PointXYZHSVtoXYZRGB(MakePointXYZHSV(h, v), p_rgb);
    return p_rgb;
}

}  // namespace nie