/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>
#include <utility>
// Local includes

#include <nie/lidar/io/network_packet_producer.hpp>
#include <nie/lidar/io/pcap_packet_producer.hpp>
#include <nie/lidar/io/streamer.hpp>

#include "radar_packet_consumer.hpp"

namespace nie::io::radar {

using PcapFileStreamer = Streamer<PcapFileProducer<Packet>, PacketConsumer>;
using NetworkStreamer = Streamer<NetworkProducer<Packet>, PacketConsumer>;

inline PcapFileStreamer CreatePcapFileStreamer(std::vector<boost::filesystem::path> source_paths) {
    auto producer = PcapFileProducer<Packet>{std::move(source_paths)};
    auto consumer = PacketConsumer();
    return {std::move(producer), std::move(consumer)};
}

inline NetworkStreamer CreateNetworkStreamer(std::string const& ip_address, std::uint16_t const& port) {
    auto producer = NetworkProducer<Packet>{ip_address, port};
    auto consumer = PacketConsumer();
    return {std::move(producer), std::move(consumer)};
}

}  // namespace nie::io::radar
