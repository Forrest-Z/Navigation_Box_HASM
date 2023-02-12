/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// Local includes
#include "kitti_packet_consumer.hpp"
#include "kitti_packet_producer.hpp"
#include "nie/lidar/io/streamer.hpp"

namespace nie {

namespace io {

namespace kitti {

using TextFileStreamer = Streamer<TextFileProducer, PacketConsumer>;

inline TextFileStreamer CreateTextFileStreamer(std::vector<boost::filesystem::path> source_paths) {
    return {TextFileProducer{std::move(source_paths)}, {}};
}

}  // namespace kitti

}  // namespace io

}  // namespace nie
