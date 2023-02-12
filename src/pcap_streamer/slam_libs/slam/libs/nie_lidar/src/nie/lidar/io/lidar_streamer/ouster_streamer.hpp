/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>
#include <utility>
// Local includes
#include "nie/lidar/io/network_packet_producer.hpp"
#include "nie/lidar/io/pcap_packet_producer.hpp"
#include "nie/lidar/io/streamer.hpp"
#include "ouster_packet_consumer.hpp"

namespace nie {

namespace io {

namespace ouster {

using PcapFileStreamer = Streamer<PcapFileProducer<Packet>, PacketConsumer>;
using NetworkStreamer = Streamer<NetworkProducer<Packet>, PacketConsumer>;

inline PcapFileStreamer CreatePcapFileStreamer(
        LidarCalibration lidar_calib, std::vector<boost::filesystem::path> source_paths) {
    auto producer = PcapFileProducer<Packet>{std::move(source_paths)};
    auto consumer = PacketConsumer{std::move(lidar_calib)};
    return {std::move(producer), std::move(consumer)};
}

inline PcapFileStreamer CreatePcapFileStreamer(
        std::string const& intr_path, std::vector<boost::filesystem::path> source_paths) {
    return CreatePcapFileStreamer(LoadCalibrationFromFile(intr_path), std::move(source_paths));
}

inline NetworkStreamer CreateNetworkStreamer(
        LidarCalibration lidar_calib, std::string const& ip_address, std::uint16_t const& port) {
    auto producer = NetworkProducer<Packet>{ip_address, port};
    auto consumer = PacketConsumer{std::move(lidar_calib)};
    return {std::move(producer), std::move(consumer)};
}

inline NetworkStreamer CreateNetworkStreamer(
        std::string const& intr_path, std::string const& ip_address, std::uint16_t const& port) {
    return CreateNetworkStreamer(LoadCalibrationFromFile(intr_path), ip_address, port);
}

}  // namespace ouster

}  // namespace io

}  // namespace nie
