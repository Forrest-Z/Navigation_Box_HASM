/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph.hpp>

#include "nie/lidar/io/filenamer.hpp"
#include "nie/lidar/pose_bbox_calculator.hpp"

namespace nie {

class LidarBboxWriter {
public:
    LidarBboxWriter(Filenamer const& filenamer, PoseBboxCalculator const& calculator);

    void ProcessSlice(Eigen::Vector3d const& offset, std::vector<Timestamp_ns> const& timestamps);

private:
    Filenamer const& las_filenamer_;
    PoseBboxCalculator const& bounds_calculator_;

    io::PoseId pose_id_;

    // Cartesian bounding box results
    io::PoseCollectionStreamWriter c_pose_writer_;
    io::BboxCollectionStreamWriter c_bbox_writer_;
    io::InfoRefCollectionStreamWriter c_iref_writer_;

    // Oriented bounding box results
    io::PoseCollectionStreamWriter o_pose_writer_;
    io::BboxCollectionStreamWriter o_bbox_writer_;
    io::InfoRefCollectionStreamWriter o_iref_writer_;
};

}  // namespace nie
