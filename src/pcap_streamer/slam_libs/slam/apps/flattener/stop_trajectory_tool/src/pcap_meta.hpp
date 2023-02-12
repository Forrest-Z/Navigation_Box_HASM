/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/time.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

namespace nie {

struct PcapMeta {
    explicit PcapMeta(boost::filesystem::path filepath) : filepath{std::move(filepath)} {}

    boost::filesystem::path filepath;
    size_t packet_count{0};
    size_t empty_packet_count{0};
    size_t point_count{0};
    Timestamp_ns begin_time;
    Timestamp_ns end_time;
    Timestamp_ns::duration max_time_delta_between_packets{std::numeric_limits<Timestamp_ns::duration::rep>::min()};
};

}  // namespace nie
