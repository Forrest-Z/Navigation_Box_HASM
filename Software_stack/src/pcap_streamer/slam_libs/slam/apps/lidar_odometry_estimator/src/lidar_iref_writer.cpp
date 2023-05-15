/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_iref_writer.hpp"

namespace nie {

LidarIrefWriter::LidarIrefWriter(Filenamer const& las_filenamer)
    : las_filenamer_{las_filenamer},
      iref_collection_writer_{las_filenamer_.FullBasename() + io::graph::Extension<io::InfoRefCollection>(), {}} {}

void LidarIrefWriter::ProcessSweep(io::PoseRecord const& pose) {
    decltype(io::InfoRefRecord::frame_id) constexpr kFrameId = 0;
    // TODO(TB)
    // MMS-1395: Using relative paths causes many problems down the road, this has to be re-evaluated
    // auto const relative_las_path = boost::filesystem::relative(las_filenamer_.Peek(), iref_parent_path_);
    iref_collection_writer_.Write(io::InfoRefRecord{pose.id, kFrameId, las_filenamer_.Peek().string()});
}

}  // namespace nie
