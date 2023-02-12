/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_bbox_writer.hpp"

namespace {

nie::io::PoseHeader CreatePoseHeader() {
    nie::io::PoseHeader header{};
    header.Set(nie::io::PoseHeader::kHasTimestampPerRecord);
    nie::io::SetNieAuthority(&header);
    return header;
}

}  // namespace

namespace nie {

LidarBboxWriter::LidarBboxWriter(Filenamer const& filenamer, PoseBboxCalculator const& calculator)
    : las_filenamer_{filenamer},
      bounds_calculator_{calculator},
      pose_id_(0),
      // Cartesian bounding box results
      c_pose_writer_{
              las_filenamer_.FullBasename() + "_cart_bbox" + io::graph::Extension<io::PoseCollection>(),
              CreatePoseHeader()},
      c_bbox_writer_{las_filenamer_.FullBasename() + "_cart_bbox" + io::graph::Extension<io::BboxCollection>(), {}},
      c_iref_writer_{las_filenamer_.FullBasename() + "_cart_bbox" + io::graph::Extension<io::InfoRefCollection>(), {}},
      // Oriented bounding box results
      o_pose_writer_{
              las_filenamer_.FullBasename() + "_orien_bbox" + io::graph::Extension<io::PoseCollection>(),
              CreatePoseHeader()},
      o_bbox_writer_{las_filenamer_.FullBasename() + "_orien_bbox" + io::graph::Extension<io::BboxCollection>(), {}},
      o_iref_writer_{
              las_filenamer_.FullBasename() + "_orien_bbox" + io::graph::Extension<io::InfoRefCollection>(), {}} {}

void LidarBboxWriter::ProcessSlice(Eigen::Vector3d const& offset, std::vector<Timestamp_ns> const& timestamps) {
    // NOTE: The timestamp written is the median timestamp used for doing things like REL TO ABS. Having this
    // information available up front simplifies several steps in the LiDAR SLAM pipeline.
    auto const time = timestamps[timestamps.size() / 2];

    // Cartesian bounding box results
    auto c_bounds = bounds_calculator_.GetBounds();
    c_bounds.origin().translation() += offset;
    c_pose_writer_.Write(io::PoseRecord{pose_id_, io::PoseRecord::Category::kBbox, {}, time, c_bounds.origin(), {}});
    c_bbox_writer_.Write(
            io::BboxRecord{pose_id_, c_bounds.bbox().min.cast<double>(), c_bounds.bbox().max.cast<double>()});
    c_iref_writer_.Write(io::InfoRefRecord{pose_id_, {}, las_filenamer_.Peek().string()});

    // Oriented bounding box results
    auto o_bounds = bounds_calculator_.GetOrientedBounds();
    o_bounds.origin().translation() += offset;
    o_pose_writer_.Write(io::PoseRecord{pose_id_, io::PoseRecord::Category::kBbox, {}, time, o_bounds.origin(), {}});
    o_bbox_writer_.Write(
            io::BboxRecord{pose_id_, o_bounds.bbox().min.cast<double>(), o_bounds.bbox().max.cast<double>()});
    o_iref_writer_.Write(io::InfoRefRecord{pose_id_, {}, las_filenamer_.Peek().string()});

    ++pose_id_;
}

}  // namespace nie
