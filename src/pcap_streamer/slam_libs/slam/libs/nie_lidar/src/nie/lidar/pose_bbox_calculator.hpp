/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <memory>

#include "helper_cloud.hpp"
#include "io/lidar_streamer.hpp"
#include "nie/lidar/geometry/pose_bbox.hpp"

namespace nie {

class PoseBboxCalculator {
public:
    template <typename PointT>
    void Calculate(pcl::PointCloud<PointT> const& cloud) {
        o_bounds_ = CalculateOrientedBounds(cloud);
        auto c_origin = o_bounds_.origin().translation();
        auto c_bbox = Bboxf::Create(cloud) - c_origin.cast<float>();
        c_bounds_ = {nie::Isometry3qd::FromTranslation(std::move(c_origin)), std::move(c_bbox)};
    }

    [[nodiscard]] PoseBbox const& GetOrientedBounds() const { return o_bounds_; };
    [[nodiscard]] PoseBbox const& GetBounds() const { return c_bounds_; };

private:
    PoseBbox o_bounds_;
    PoseBbox c_bounds_;
};

}  // namespace nie
