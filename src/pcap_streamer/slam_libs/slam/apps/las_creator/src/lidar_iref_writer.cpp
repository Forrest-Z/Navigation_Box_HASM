/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "lidar_iref_writer.hpp"

namespace nie {

LidarIrefWriter::LidarIrefWriter(Filenamer const& las_filenamer, std::vector<io::PoseRecord> const& poses)
    : las_filenamer_{las_filenamer},
      poses_{poses},
      iref_collection_writer_{las_filenamer_.FullBasename() + io::graph::Extension<io::InfoRefCollection>(), {}} {}

void LidarIrefWriter::ProcessSlice(std::vector<Timestamp_ns> const& timestamps) {
    if (timestamps.empty()) return;

    // First that is equal to or bigger than timestamp.
    auto it_begin = std::lower_bound(poses_.cbegin(), poses_.cend(), timestamps.front());
    auto it_end = std::upper_bound(it_begin, poses_.cend(), timestamps.back());
    // If it is equal we move the iterator one place back to avoid duplicate ids with another las file.
    if (timestamps.back() == it_end->timestamp) {
        it_end--;
    }

    for (auto it = it_begin; it != it_end; ++it) {
        iref_collection_writer_.Write(
                io::InfoRefRecord{it->id, decltype(io::InfoRefRecord::frame_id){}, las_filenamer_.Peek().string()});
    }
}

}  // namespace nie
