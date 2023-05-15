// Copyright 2020 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Changed namespace from autoware to aiim
// * Adding aiim prefix to includes
// * Changed dependencies

#include <aiim_autoware_common/types.hpp>
#include <aiim_autoware_msgs/msg/bounding_box_array.hpp>
#include <aiim_motion_common/config.hpp>
#include <aiim_time_utils/time_utils.hpp>
#include <algorithm>
#include <memory>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifndef OBJECT_COLLISION_ESTIMATOR_NODES__VISUALIZE_HPP_
#define OBJECT_COLLISION_ESTIMATOR_NODES__VISUALIZE_HPP_

namespace aiim {
namespace planning {
namespace object_collision_estimator_nodes {

using aiim::common::types::float32_t;
using aiim::common::types::float64_t;
using aiim_autoware_msgs::msg::BoundingBoxArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

MarkerArray toVisualizationMarkerArray(const BoundingBoxArray bboxes, const size_t collision_idx) {
    MarkerArray marker_array{};

    // delete previous markers
    // TODO(mitsudome-r): remove delete_marker once lifetime is supported by rviz
    Marker delete_marker{};
    delete_marker.header = bboxes.header;
    delete_marker.ns = "bounding_box";
    delete_marker.action = Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // create markers for bounding boxes
    Marker marker{};
    marker.header = bboxes.header;
    marker.ns = "bounding_box";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.lifetime = aiim::common::time_utils::to_message(std::chrono::nanoseconds(100000));

    for (std::size_t i = 0; i < bboxes.boxes.size(); ++i) {
        marker.id = static_cast<int32_t>(i);
        marker.pose.orientation.w = 1.0;
        if (i < collision_idx) {
            marker.scale.x = 0.2;
            marker.color.a = 0.3F;
            marker.color.r = 0.0F;
            marker.color.g = 1.0F;
            marker.color.b = 0.0F;
        } else if (i == collision_idx) {
            marker.scale.x = 0.2;
            marker.color.a = 1.0F;
            marker.color.r = 1.0F;
            marker.color.g = 0.0F;
            marker.color.b = 0.0F;
        } else {
            marker.scale.x = 0.2;
            marker.color.a = 0.3F;
            marker.color.r = 0.7F;
            marker.color.g = 0.7F;
            marker.color.b = 0.7F;
        }
        marker.points.clear();
        const auto box = bboxes.boxes.at(i);
        for (std::size_t j = 0; j < 4; ++j) {
            geometry_msgs::msg::Point point;
            point.x = static_cast<float64_t>(box.corners.at(j).x);
            point.y = static_cast<float64_t>(box.corners.at(j).y);
            point.z = static_cast<float64_t>(box.corners.at(j).z);
            marker.points.push_back(point);
        }
        geometry_msgs::msg::Point point;
        point.x = static_cast<float64_t>(box.corners.at(0).x);
        point.y = static_cast<float64_t>(box.corners.at(0).y);
        point.z = static_cast<float64_t>(box.corners.at(0).z);
        marker.points.push_back(point);

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}
}  // namespace object_collision_estimator_nodes
}  // namespace planning
}  // namespace aiim

#endif  // OBJECT_COLLISION_ESTIMATOR_NODES__VISUALIZE_HPP_
