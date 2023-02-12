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
};

void PrintPcapMeta(PcapMeta const& pcap_meta) {
    LOG(INFO) << "filepath = " << pcap_meta.filepath;
    LOG(INFO) << "packet count = " << pcap_meta.packet_count;
    LOG(INFO) << "empty packet count = " << pcap_meta.empty_packet_count;
    LOG(INFO) << "point count = " << pcap_meta.point_count;
    LOG(INFO) << "begin time = " << pcap_meta.begin_time;
    LOG(INFO) << "end time = " << pcap_meta.end_time;
}

}  // namespace nie
